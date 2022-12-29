#!/usr/bin/env python3

from robotic_hw_solution.srv import Sensor
from std_msgs.msg import Float64MultiArray, MultiArrayLayout, MultiArrayDimension

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import numpy as np
import socket
import threading
import numbers

class SensorServiceServer(Node):
    # Class that makes calls to sensor to read data, filters the data,
    # and provides a ROS service to read the latest filtered data
    def __init__(self):
        super().__init__('sensor_service_server')

        # Declare parameters and default values
        # 'host' - sensor ip address 
        # 'port' - sensor port
        # 'sampling_freq' - sampling frequency of sensor in Hz
        # 'delay' - delay (overhead) per call to sensor in seconds
        # 'noise_process' - Process noise covariance. Additive noise per second.
        #                   Used for calculating filter weights and optimal
        #                   number of samples. 
        # 'noise_sensor' - Sensor noise covariance. Noise per sample. Used for 
        #                  calculating filter weights and optimal number of 
        #                  samples.
        # 'min_update_freq' - Minimum update frequency for sensor. Places a
        #                     constraint on the number of samples per call so 
        #                     that updated filtered data will be available at
        #                     this minimum frequency on average.
        self.declare_parameters(
            namespace='',
            parameters=[
                ('host', '127.0.0.3'),
                ('port', 10000),
                ('sampling_freq', 2000),
                ('delay', 0.0015),
                ('noise_process', 1.0),
                ('noise_sensor', 0.05),
                ('min_update_freq', 100.0)
            ]
        )

        # Get parameters, specifying min/max values where appropriate
        sampling_freq = self.getParam('sampling_freq', min_value=1.e-6,
                                                            max_value=10000)
        self.dt = 1.0/sampling_freq
        self.delay = self.getParam('delay', min_value=0.0)
        self.noise_process = self.getParam('noise_process', min_value=1.e-6)
        self.noise_sensor = self.getParam('noise_sensor', min_value=1.e-6)
        self.min_update_freq = self.getParam('min_update_freq', min_value=0.1)

        # Set up socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        host = self.getParam('host')
        port = self.getParam('port')
        server_address = (host, port)

        # Try to connect. If cannot connect, return
        self.get_logger().info('connecting to {} port {}'.format(*server_address))
        try:
            self.sock.connect(server_address)
        except socket.error as e:
            self.get_logger().error("Error (%s) while connecting. Aborting." % e)
            return

        # Calculate optimal number of samples

        # Calculate maximum number of samples per call that can achieve an
        # update frequency of at least min_update_freq. 
        N_max = int((1.0/self.min_update_freq - self.delay)*sampling_freq)

        # Ensure at least 1 sample per call. This leads to violating
        # min_update_freq in cases where it is not possible to receive a single
        # sample in time to achieve min_update_freq
        if N_max < 1:
            N_max = 1
            update_freq_max = 1.0/(self.dt + self.delay)
            self.get_logger().warn("Cannot achieve desired minimum update "
                                    "frequency (%.1f Hz) for sensor with "
                                    "sampling frequency (%.1f Hz) and delay "
                                    "(%.4f s). Defaulting to 1 sample at a "
                                    "time, which can achieve update frequency "
                                    "(%.1f Hz)" % (self.min_update_freq, \
                                    sampling_freq, self.delay, update_freq_max))
        
        # Find the number of samples per call that minimizes the average
        # error covariance (cov_avg_min) of the filtered data in steady state.
        # Model as a random process where the state evolves with additive 
        # white noise (noise_process) over time with no bias. Model sensor
        # measurement noise as constant white noise (noise_sensor). Penalize 
        # older measurements by assuming the effective measurement noise 
        # includes the process noise added during the time between the sample 
        # and the update (noise_effective = noise_sensor + noise_process*t
        # where t is the age of the sample in seconds). Loop over all possible 
        # number of samples per call (N), calculate the average steady state
        # error covariance (cov_avg_N) for each, and choose N that minimizes
        # cov_avg 
        cov_avg_min = None 
        T = None
        N_optimal = None
        noise_inv_arr = np.array([]) # Keep track of the noise for each sample
                                     # while looping to calculate weights later
        for N in range(1, N_max + 1):
            T_N = N*self.dt + self.delay # Total time to receive N samples
            # Calculate (reciprical of) the effective measurement noise for
            # the oldest sample in the batch (N samples old). 
            noise_inv = 1.0/(self.noise_sensor + self.noise_process*self.dt*N)
            noise_inv_arr = np.append(noise_inv_arr, noise_inv)
            # Calculate the average steady state error covariance when sampling
            # N samples per call
            cov_avg_N = 0.5*self.noise_process*T_N*np.sqrt(1.0 + \
                            4.0/(self.noise_process*T_N*noise_inv_arr.sum()))
            # Check if this error covariance is smaller than for all previous N
            if cov_avg_min is None or cov_avg_N < cov_avg_min:
                # Update relevant variables if so
                cov_avg_min = cov_avg_N
                T = T_N
                N_optimal = N

        # Calculate the steady state apriori error covariance. Since the average
        # steady state error covariance happens halfway through the total
        # sampling period T, simply add 0.5*noise_process*T to cov_avg_min
        cov_apriori = cov_avg_min + 0.5*self.noise_process*T

        # Calculate filter weights by appending (the reciprocal of) the apriori
        # error covariance to (the reciprocal of) the effective sensor measurement 
        # noise for each sample and then normalizing so the weights sum to 1
        weights = np.append(1.0/cov_apriori, noise_inv_arr[:N_optimal])
        weights /= weights.sum()

        self.w0 = weights[0] # w0 is the weight for the prior estimate
        self.w = weights[1:] # w is the weights for the new samples

        self.num_samples = N_optimal # Use optimal number of samples

        # Initialize filtered data. The choice of initializing with 0's will 
        # bias the filter towards 0 initially, but the bias will diminish in
        # steady state
        self.filtered_data = np.zeros(6)
        # Set up layout for MultiArray since it is the same layout for each 
        # service call
        self.data_layout = MultiArrayLayout()
        self.data_layout.data_offset = 0
        dim = MultiArrayDimension()
        dim.stride = 1
        dim.size = 6
        self.data_layout.dim.append(dim)

        # Use lock to avoid two threads accessing filtered data simultaneously
        self.data_lock = threading.Lock()

        # Start timer for continuously reading data from sensor and updating
        # filtered data. Start service for reading the latest filtered data
        # Use separate callback groups so that the two threads can run in 
        # parallel 
        self.cb_group_timer = MutuallyExclusiveCallbackGroup()
        self.cb_group_srv = MutuallyExclusiveCallbackGroup()
        self.update_timer = self.create_timer(0.0, self.update_sensor,
                                            callback_group=self.cb_group_timer)
        self.read_sensor_srv = self.create_service(Sensor, 'read_sensor',
                        self.read_sensor_cb, callback_group=self.cb_group_srv)

    def read_sensor_cb(self, request, response):
        # Service call back for reading the latest filtered data
        response.data.layout = self.data_layout
        with self.data_lock:
            response.data.data = list(self.filtered_data)
        return response

    def update_sensor(self):
        # Timer callback for reading data from sensor and updating filtered data

        # Send message to sensor to receive samples
        message_string = str(self.num_samples)
        message = message_string.encode()
        self.sock.sendall(message)

        # Receive samples
        byte_data = self.sock.recv(10000)
        data =  np.frombuffer(byte_data).reshape(6, -1).astype(float)

        # Update filtered data using filter weights
        with self.data_lock:
            self.filtered_data = self.filtered_data*self.w0 + data @ self.w

    def close(self):
        # Function for closing socket
        self.get_logger().info('closing socket')
        self.sock.close()

    def getParam(self, name, min_value=None, max_value=None):
        # Helper function for getting parameter by name.
        # For numeric parameters (float, int, etc), can optionally specify
        # min value or max value of the parameter
        value = self.get_parameter(name).value
        if isinstance(value, numbers.Number):
            if isinstance(min_value, numbers.Number) and value < min_value:
                self.get_logger().warn("Value of parameter %s (%f) is less "
                                    "than minimum parameter value (%f). "
                                    "Defaulting to minimum parameter value" % \
                                    (name, value, min_value))
                value = min_value
            if isinstance(max_value, numbers.Number) and value > max_value:
                self.get_logger().warn("Value of parameter %s (%f) is greater "
                                    "than maximum parameter value (%f). "
                                    "Defaulting to maximum parameter value" % \
                                    (name, value, max_value))
                value = max_value

        return value

def main(args=None):
    rclpy.init(args=args)

    sensor_service_srv = SensorServiceServer()

    # Use MultiThreadedExecutor so that threads can run in parallel
    executor = MultiThreadedExecutor()
    executor.add_node(sensor_service_srv)
    executor.spin()

    sensor_service_srv.close()  # close the socket

    rclpy.shutdown()

if __name__ == '__main__':
    main()