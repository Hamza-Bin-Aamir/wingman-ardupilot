from pymavlink import mavutil
import threading
import queue
import time
import fnmatch

class ParameterManager:
    """
    Parameter manager for ArduPilot vehicles.
    Handles parameter reading, caching, and searching functionality.
    """
    
    def __init__(self, vehicle):
        """
        Initialize parameter manager with a vehicle instance.
        
        Args:
            vehicle: Vehicle instance that provides MAVLink connection
        """
        self.vehicle = vehicle
        self._param_lock = threading.Lock()
        self.param_cache = {}                       # Cache for PARAM_VALUE responses {param_id: param_value}
        self.pending_param_requests = set()         # Track pending PARAM_REQUEST_READ requests
        self.param_list_complete = False            # Track if we've received all parameters
        self.total_param_count = 0                  # Total number of parameters on vehicle
        
    def request_parameter_list(self, timeout=30, target_system=1, target_component=1):
        """
        Request all parameters from the vehicle and cache them.
        
        Args:
            timeout: Maximum time to wait for all parameters
            target_system: Target system ID
            target_component: Target component ID
            
        Returns:
            bool: True if successful, False if timeout
        """
        with self._param_lock:
            # Clear existing cache
            self.param_cache.clear()
            self.param_list_complete = False
            self.total_param_count = 0
            
            # Register for PARAM_VALUE messages
            param_queue = self.vehicle.register_message_listener('PARAM_VALUE')
            
            # Request parameter list
            self.vehicle.master.mav.param_request_list_send(target_system, target_component)
            
            start_time = time.time()
            received_count = 0
            
            try:
                while time.time() - start_time < timeout:
                    try:
                        msg = param_queue.get(timeout=1.0)
                        if msg:
                            # Extract parameter info
                            param_id = msg.param_id
                            if isinstance(param_id, bytes):
                                param_id = param_id.decode('utf-8').rstrip('\x00')
                            elif isinstance(param_id, str):
                                param_id = param_id.rstrip('\x00')
                            
                            param_value = msg.param_value
                            param_index = msg.param_index
                            param_count = msg.param_count
                            
                            # Update total count
                            if self.total_param_count == 0:
                                self.total_param_count = param_count
                            
                            # Cache the parameter
                            self.param_cache[param_id] = {
                                'value': param_value,
                                'index': param_index,
                                'type': msg.param_type
                            }
                            
                            received_count += 1
                            
                            # Check if we have all parameters
                            if received_count >= param_count:
                                self.param_list_complete = True
                                break
                                
                    except queue.Empty:
                        continue
                        
            finally:
                self.vehicle.unregister_message_listener('PARAM_VALUE', param_queue)
            
            return self.param_list_complete
    
    def get_parameter(self, param_name, timeout=5, target_system=1, target_component=1):
        """
        Get a specific parameter value.
        
        Args:
            param_name: Name of the parameter to get
            timeout: Maximum time to wait for response
            target_system: Target system ID  
            target_component: Target component ID
            
        Returns:
            Parameter value or None if not found/timeout
        """
        # Check cache first
        with self._param_lock:
            if param_name in self.param_cache:
                return self.param_cache[param_name]['value']
        
        # Request specific parameter
        param_queue = self.vehicle.register_message_listener('PARAM_VALUE')
        
        try:
            # Send parameter request
            if isinstance(param_name, str):
                param_name_bytes = param_name.encode('utf-8')[:16]  # Max 16 chars
            else:
                param_name_bytes = param_name[:16]  # Already bytes
                
            self.vehicle.master.mav.param_request_read_send(
                target_system,
                target_component,
                param_name_bytes,
                -1  # param_index (-1 means use param_id)
            )
            
            start_time = time.time()
            while time.time() - start_time < timeout:
                try:
                    msg = param_queue.get(timeout=1.0)
                    if msg:
                        received_param_id = msg.param_id
                        if isinstance(received_param_id, bytes):
                            received_param_id = received_param_id.decode('utf-8').rstrip('\x00')
                        elif isinstance(received_param_id, str):
                            received_param_id = received_param_id.rstrip('\x00')
                            
                        if received_param_id == param_name:
                            param_value = msg.param_value
                            # Cache the parameter
                            with self._param_lock:
                                self.param_cache[param_name] = {
                                    'value': param_value,
                                    'index': msg.param_index,
                                    'type': msg.param_type
                                }
                            return param_value
                except queue.Empty:
                    continue
                    
        finally:
            self.vehicle.unregister_message_listener('PARAM_VALUE', param_queue)
        
        return None
    
    def set_parameter(self, param_name, param_value, timeout=5, target_system=1, target_component=1):
        """
        Set a specific parameter value.
        
        Args:
            param_name: Name of the parameter to set
            param_value: New value for the parameter
            timeout: Maximum time to wait for response
            target_system: Target system ID  
            target_component: Target component ID
            
        Returns:
            bool: True if parameter was set successfully, False otherwise
        """
        # Register for PARAM_VALUE messages to confirm the change
        param_queue = self.vehicle.register_message_listener('PARAM_VALUE')
        
        try:
            # Send parameter set command
            if isinstance(param_name, str):
                param_name_bytes = param_name.encode('utf-8')[:16]  # Max 16 chars
            else:
                param_name_bytes = param_name[:16]  # Already bytes
            
            # Convert value to float (ArduPilot parameters are typically float)
            try:
                param_value_float = float(param_value)
            except (ValueError, TypeError):
                return False
                
            self.vehicle.master.mav.param_set_send(
                target_system,
                target_component,
                param_name_bytes,
                param_value_float,
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32  # Most common parameter type
            )
            
            start_time = time.time()
            while time.time() - start_time < timeout:
                try:
                    msg = param_queue.get(timeout=1.0)
                    if msg:
                        received_param_id = msg.param_id
                        if isinstance(received_param_id, bytes):
                            received_param_id = received_param_id.decode('utf-8').rstrip('\x00')
                        elif isinstance(received_param_id, str):
                            received_param_id = received_param_id.rstrip('\x00')
                            
                        if received_param_id == param_name:
                            # Parameter was set, update cache
                            with self._param_lock:
                                self.param_cache[param_name] = {
                                    'value': msg.param_value,
                                    'index': msg.param_index,
                                    'type': msg.param_type
                                }
                            return True
                except queue.Empty:
                    continue
                    
        finally:
            self.vehicle.unregister_message_listener('PARAM_VALUE', param_queue)
        
        return False
    
    def search_parameters(self, pattern):
        """
        Search for parameters matching a pattern (supports wildcards).
        
        Args:
            pattern: Search pattern (e.g., "GFSD_*", "*_ENABLE", "GPS*")
            
        Returns:
            dict: Dictionary of matching parameters {param_name: param_info}
        """
        with self._param_lock:
            if not self.param_cache:
                return {}
            
            matching_params = {}
            pattern_upper = pattern.upper()
            
            for param_name, param_info in self.param_cache.items():
                param_name_upper = param_name.upper()
                if fnmatch.fnmatch(param_name_upper, pattern_upper):
                    matching_params[param_name] = param_info
            
            return matching_params
    
    def get_all_parameters(self):
        """
        Get all cached parameters.
        
        Returns:
            dict: Dictionary of all parameters {param_name: param_info}
        """
        with self._param_lock:
            return self.param_cache.copy()
    
    def get_parameter_count(self):
        """
        Get the total number of parameters and cached count.
        
        Returns:
            tuple: (total_count, cached_count)
        """
        with self._param_lock:
            return self.total_param_count, len(self.param_cache)
    
    def is_cache_complete(self):
        """
        Check if parameter cache is complete.
        
        Returns:
            bool: True if all parameters have been cached
        """
        with self._param_lock:
            return self.param_list_complete
