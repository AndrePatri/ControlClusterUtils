# Copyright (C) 2023  Andrea Patrizi (AndrePatri, andreapatrizi1b6e6@gmail.com)
# 
# This file is part of CoClusterBridge and distributed under the General Public License version 2 license.
# 
# CoClusterBridge is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 2 of the License, or
# (at your option) any later version.
# 
# CoClusterBridge is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with CoClusterBridge.  If not, see <http://www.gnu.org/licenses/>.
# 
from abc import ABC, abstractmethod

import time 

from control_cluster_bridge.utilities.shared_data.rhc_data import RobotState
from control_cluster_bridge.utilities.shared_data.rhc_data import RhcCmds
from control_cluster_bridge.utilities.shared_data.rhc_data import RhcStatus
from control_cluster_bridge.utilities.shared_data.rhc_data import RhcInternal
from control_cluster_bridge.utilities.shared_data.cluster_profiling import RhcProfiling

from control_cluster_bridge.utilities.defs import Journal
from control_cluster_bridge.utilities.homing import RobotHomer

from SharsorIPCpp.PySharsorIPC import VLevel
from SharsorIPCpp.PySharsorIPC import Journal, LogType

from typing import List, TypeVar

import torch
import numpy as np

from perf_sleep.pyperfsleep import PerfSleep

class CntrlCmd(ABC):

    pass

class RHCCmd(ABC):

    pass

RHCCmdChild = TypeVar('RHCCmdChild', bound='RHCCmd')

class RHController(ABC):

    def __init__(self, 
            cluster_size: int,
            controller_index: int,
            srdf_path: str,
            n_nodes: int,
            verbose = False, 
            debug = False,
            array_dtype = torch.float32, 
            namespace = "",
            debug_sol = False):
        
        self.namespace = namespace
        
        self.perf_timer = PerfSleep()

        self.controller_index = controller_index
        self.controller_index_torch = torch.tensor(controller_index)

        self.srdf_path = srdf_path

        self._verbose = verbose
        self._debug = debug
        self._debug_sol = debug_sol

        self._profiling_data_dict = {}
        self._profiling_data_dict["full_solve_dt"] = np.nan
        self._profiling_data_dict["rti_solve_dt"] = np.nan
        self._profiling_data_dict["problem_update_dt"] = np.nan
        self._profiling_data_dict["phases_shift_dt"] = np.nan
        self._profiling_data_dict["task_ref_update"] = np.nan
        
        self.cluster_size = cluster_size

        self.n_dofs = None
        self.n_contacts = None
        
        # shared mem
        self.robot_state = None 

        self.rhc_status = None
        self.rhc_internal = None
        self.cluster_stats = None

        self.robot_cmds = None
        self.rhc_task_refs = None

        # jnt names
        self._env_side_jnt_names = []
        self._controller_side_jnt_names = []

        self._got_jnt_names_from_controllers = False

        # data maps
        self._to_controller = []
        self._quat_remap = [0, 1, 2, 3] # defaults to no remap (to be overridden)
        self._jnt_maps_created = False
        
        self._states_initialized = False

        self._trigger_flag = False

        self.array_dtype = array_dtype

        self.add_data_lenght = 0

        self.n_resets = 0
        self.n_fails = 0

        self._n_nodes = n_nodes

        self._start_time = time.perf_counter()

        self._homer: RobotHomer = None

        self._init()

    def __del__(self):
        
        self._terminate()

    def _terminate(self):

        if self.robot_cmds is not None:
            
            self.robot_cmds.close()
        
        if self.robot_state is not None:
            
            self.robot_state.close()
        
        if self.rhc_status is not None:
            
            # signal controller deactivation over shared mem
            self.rhc_status.activation_state.write_wait(False, 
                                    row_index=self.controller_index,
                                    col_index=0)
            
            # decrementing controllers counter
            self.rhc_status.controllers_counter.synch_all(wait = True,
                                                    read = True)
            self.rhc_status.controllers_counter.torch_view -= 1 
            self.rhc_status.controllers_counter.synch_all(wait = True,
                                                    read = False)
        
            self.rhc_status.close()
        
        if self.rhc_internal is not None:

            self.rhc_internal.close()

        if self.cluster_stats is not None:

            self.cluster_stats.close()

    def _init(self):

        self._init_problem() # we call the child's initialization method

        self.rhc_status = RhcStatus(is_server=False,
                                    namespace=self.namespace, 
                                    verbose=True, 
                                    vlevel=VLevel.V2)

        self.rhc_status.run()

        # incrementing controllers counter
        self.rhc_status.controllers_counter.synch_all(wait = True,
                                                read = True)
        self.rhc_status.controllers_counter.torch_view += 1 
        self.rhc_status.controllers_counter.synch_all(wait = True,
                                                read = False)
        
        if self._debug_sol:
            
            # internal solution is published on shared mem

            # we assume the user has made available the cost
            # and constraint data at this point (e.g. through
            # the solution of a bootstrap)
             
            cost_data = self._get_cost_data()
            constr_data = self._get_constr_data()

            config = RhcInternal.Config(is_server=True, 
                        enable_q= True, 
                        enable_v=False, 
                        enable_a=False, 
                        enable_a_dot=True, 
                        enable_f=True,
                        enable_f_dot=True, 
                        enable_eff=True, 
                        cost_names=cost_data[0], 
                        cost_dims=cost_data[1],
                        constr_names=constr_data[0],
                        constr_dims=constr_data[1],
                        )
            
            self.rhc_internal = RhcInternal(config=config, 
                                    namespace=self.namespace,
                                    rhc_index = self.controller_index,
                                    is_server=True, 
                                    n_contacts=self.n_contacts,
                                    n_jnts=self.n_dofs,
                                    n_nodes=self._n_nodes,
                                    verbose = self._verbose,
                                    vlevel=VLevel.V2)
            
            self.rhc_internal.run()
            
            # statistical data

            self.cluster_stats = RhcProfiling(cluster_size=self.cluster_size,
                                        is_server=False, 
                                        name=self.namespace,
                                        verbose=self._verbose,
                                        vlevel=VLevel.V2,
                                        safe=True)

            self.cluster_stats.run()
            
        self._homer = RobotHomer(self.srdf_path, 
                            self._controller_side_jnt_names)
    
    def _update_profiling_data(self):

        # updated debug data on shared memory
        # with the latest info available
        self.cluster_stats.solve_loop_dt.write_wait(self._profiling_data_dict["full_solve_dt"], 
                                                            row_index=self.controller_index,
                                                            col_index=0)
        
        self.cluster_stats.rti_sol_time.write_wait(self._profiling_data_dict["rti_solve_dt"], 
                                                            row_index=self.controller_index,
                                                            col_index=0)
        
        self.cluster_stats.prb_update_dt.write_wait(self._profiling_data_dict["problem_update_dt"], 
                                                            row_index=self.controller_index,
                                                            col_index=0)
        
        self.cluster_stats.phase_shift_dt.write_wait(self._profiling_data_dict["phases_shift_dt"], 
                                                            row_index=self.controller_index,
                                                            col_index=0)
        
        self.cluster_stats.task_ref_update_dt.write_wait(self._profiling_data_dict["task_ref_update"], 
                                                            row_index=self.controller_index,
                                                            col_index=0)
        
    def init_rhc_task_cmds(self):
        
        self.rhc_task_refs = self._init_rhc_task_cmds()
        
    def init_states(self):

        # to be called after n_dofs is known
        self.robot_state = RobotState(namespace=self.namespace,
                                is_server=False,
                                q_remapping=self._quat_remap, # remapping from environment to controller
                                with_gpu_mirror=False,
                                safe=False,
                                verbose=self._verbose,
                                vlevel=VLevel.V2) 
        self.robot_state.run()
        
        self.robot_cmds = RhcCmds(namespace=self.namespace,
                                is_server=False,
                                q_remapping=self._quat_remap, # remapping from environment to controller
                                with_gpu_mirror=False,
                                safe=False,
                                verbose=self._verbose,
                                vlevel=VLevel.V2) 
        self.robot_cmds.run()
                
        self._states_initialized = True
        
    def solve(self):
        
        if not self._jnt_maps_created:
            
            exception = f"Jnt maps not initialized. Did you call the create_jnt_maps()?"

            Journal.log(self.__class__.__name__,
                    "solve",
                    exception,
                    LogType.EXCEP,
                    throw_when_excep = True)

        if not self._states_initialized:

            exception =f"States not initialized. Did you call the init_states()?"

            Journal.log(self.__class__.__name__,
                    "solve",
                    exception,
                    LogType.EXCEP,
                    throw_when_excep = True)

        if self.rhc_task_refs is None:

            exception = f"RHC task references non initialized. Did you call init_rhc_task_cmds()?"

            Journal.log(self.__class__.__name__,
                    "solve",
                    exception,
                    LogType.EXCEP,
                    throw_when_excep = True)
        
        while True:
            
            # we are always listening for a trigger signal from the client 
            # or a reset signal
            
            try:
                
                # checks for reset requests
                if self.rhc_status.resets.read_wait(row_index=self.controller_index,
                                            col_index=0)[0]:

                    self.reset()

                    self.fail_n = 0 # resets number of fails

                    self.rhc_status.resets.write_wait(False, 
                                                    row_index=self.controller_index,
                                                    col_index=0)

                # checks for trigger requests
                if self.rhc_status.trigger.read_wait(row_index=self.controller_index,
                                                    col_index=0)[0]:
                    
                    if self._debug:
                        
                        self._start_time = time.perf_counter()

                    self.robot_state.synch_from_shared_mem() # updates robot state with
                    # latest data on shared mem

                    # latest state is employed
                    success = self._solve() # solve actual TO

                    if (not success):
                        
                        self._on_failure()

                    self._write_cmds_from_sol() # we update update the views of the cmd
                    # from the latest solution

                    if self._debug_sol:
                        
                        # if in debug, rhc internal state is streamed over 
                        # shared mem.
                        self._update_rhc_internal()

                    # we signal the client this controller has finished its job by
                    # resetting the flag
                    self.rhc_status.trigger.write_wait(False, 
                                                    row_index=self.controller_index,
                                                    col_index=0)

                    if self._debug:
                        
                        self._profiling_data_dict["full_solve_dt"] = time.perf_counter() - self._start_time

                        self._update_profiling_data() # updates all profiling data

                    if self._verbose and self._debug:
                        
                        Journal.log(self.__class__.__name__,
                            "solve",
                            f"RHC full solve loop execution time  -> " + str(self._profiling_data_dict["full_solve_dt"]),
                            LogType.INFO,
                            throw_when_excep = True)
                
                else:
                    
                    # we avoid busy waiting and CPU saturation by sleeping for a small amount of time

                    self.perf_timer.clock_sleep(1000000) # nanoseconds (actually resolution is much
                    # poorer)

            except KeyboardInterrupt:

                break
    
    def _on_failure(self):
        
        # self.controller_fail_flag.set_bool(True) # can be read by the cluster client

        # self.reset() # resets controller (this has to be defined by the user)

        self.rhc_status.fails.write_wait(True, 
                                        row_index=self.controller_index,
                                        col_index=0)
        
        self.n_fails += 1

    def reset(self):
        
        self._reset()

        self.n_resets += 1

    def create_jnt_maps(self):
        
        # retrieve env-side joint names from shared mem
        self._env_side_jnt_names = self.robot_state.jnt_names()

        self._check_jnt_names_compatibility() # will raise exception

        if not self._got_jnt_names_from_controllers:
            
            exception = f"Cannot run the solve(). assign_env_side_jnt_names() was not called!"

            Journal.log(self.__class__.__name__,
                    "create_jnt_maps",
                    exception,
                    LogType.EXCEP,
                    throw_when_excep = True)
        
        self._to_controller = [self._env_side_jnt_names.index(element) for element in self._controller_side_jnt_names]
        
        # set joint remappings for shared data
        self.robot_state.set_jnts_remapping(jnts_remapping=self._to_controller)
        self.robot_cmds.set_jnts_remapping(jnts_remapping=self._to_controller)

        self._jnt_maps_created = True

    def set_cmds_to_homing(self):

        homing = torch.tensor(self._homer.get_homing()).reshape(1, 
                            self.robot_cmds.n_jnts())
        
        null_action = torch.zeros((1, self.robot_cmds.n_jnts()), 
                        dtype=self.array_dtype)
        
        self.robot_cmds.jnts_state.set_q(q = homing, robot_idxs=self.controller_index_torch)

        self.robot_cmds.jnts_state.set_v(v = null_action, robot_idxs=self.controller_index_torch)

        self.robot_cmds.jnts_state.set_eff(eff = null_action, robot_idxs=self.controller_index_torch)

    def _write_cmds_from_sol(self):

        # gets data from the solution and updates the view on the shared data

        self.robot_cmds.jnts_state.set_q(q = self._get_cmd_jnt_q_from_sol(), robot_idxs=self.controller_index_torch)

        self.robot_cmds.jnts_state.set_v(v = self._get_cmd_jnt_v_from_sol(), robot_idxs=self.controller_index_torch)

        self.robot_cmds.jnts_state.set_eff(eff = self._get_cmd_jnt_eff_from_sol(), robot_idxs=self.controller_index_torch)

        # self.robot_cmds.slvr_state.set_info(self._get_additional_slvr_info())

        self.robot_cmds.synch_to_shared_mem()

    def _assign_controller_side_jnt_names(self, 
                        jnt_names: List[str]):

        self._controller_side_jnt_names = jnt_names

        self._got_jnt_names_from_controllers = True

    def _check_jnt_names_compatibility(self):

        set_srvr = set(self._controller_side_jnt_names)
        set_client  = set(self._env_side_jnt_names)

        if not set_srvr == set_client:

            exception = "Server-side and client-side joint names do not match!"

            Journal.log(self.__class__.__name__,
                    "_check_jnt_names_compatibility",
                    exception,
                    LogType.EXCEP,
                    throw_when_excep = True)
    
    def _get_cost_data(self):
        
        # to be overridden by child class
        return None, None
    
    def _get_constr_data(self):
        
        # to be overridden by child class
        return None, None
    
    def _update_rhc_internal(self):

        # data which is not enabled in the config is not actually 
        # written so overhead is minimal in for non-enabled data

        self.rhc_internal.write_q(data= self._get_q_from_sol(),
                            wait=True)

        self.rhc_internal.write_v(data= self._get_v_from_sol(),
                            wait=True)
        
        self.rhc_internal.write_a(data= self._get_a_from_sol(),
                            wait=True)
        
        self.rhc_internal.write_a_dot(data= self._get_a_dot_from_sol(),
                            wait=True)
        
        self.rhc_internal.write_f(data= self._get_f_from_sol(),
                            wait=True)
        
        self.rhc_internal.write_f_dot(data= self._get_f_dot_from_sol(),
                            wait=True)
        
        self.rhc_internal.write_eff(data= self._get_eff_from_sol(),
                            wait=True)

        for cost_idx in range(self.rhc_internal.config.n_costs):
            
            # iterate over all costs and update all values
            cost_name = self.rhc_internal.config.cost_names[cost_idx]

            self.rhc_internal.write_cost(data= self._get_cost_from_sol(cost_name = cost_name),
                                cost_name = cost_name,
                                wait=True)
        
        for constr_idx in range(self.rhc_internal.config.n_constr):

            # iterate over all constraints and update all values
            constr_name = self.rhc_internal.config.constr_names[constr_idx]

            self.rhc_internal.write_constr(data= self._get_constr_from_sol(constr_name=constr_name),
                                constr_name = constr_name,
                                wait=True)
        
    def _get_q_from_sol(self):

        # to be overridden by child class

        return None

    def _get_v_from_sol(self):

        # to be overridden by child class
        
        return None
    
    def _get_a_from_sol(self):

        # to be overridden by child class
        
        return None
    
    def _get_a_dot_from_sol(self):

        # to be overridden by child class
        
        return None
    
    def _get_f_from_sol(self):

        # to be overridden by child class
        
        return None
    
    def _get_f_dot_from_sol(self):

        # to be overridden by child class
        
        return None
    
    def _get_eff_from_sol(self):

        # to be overridden by child class
        
        return None
    
    def _get_cost_from_sol(self,
                    cost_name: str):

        # to be overridden by child class
        
        return None
    
    def _get_constr_from_sol(self,
                    constr_name: str):

        # to be overridden by child class
        
        return None
    
    @abstractmethod
    def _reset(self):
        
        pass

    @abstractmethod
    def _init_rhc_task_cmds(self):

        pass

    @abstractmethod
    def _get_robot_jnt_names(self) -> List[str]:

        pass

    @abstractmethod
    def _get_cmd_jnt_q_from_sol(self) -> torch.Tensor:

        pass

    @abstractmethod
    def _get_cmd_jnt_v_from_sol(self) -> torch.Tensor:

        pass
    
    @abstractmethod
    def _get_cmd_jnt_eff_from_sol(self) -> torch.Tensor:

        pass
    
    @abstractmethod
    def _get_additional_slvr_info(self) -> torch.Tensor:

        pass

    @abstractmethod
    def _update_open_loop(self):

        # updates measured robot state 
        # using the internal robot state of the RHC controller

        pass
    
    @abstractmethod
    def _update_closed_loop(self):

        # updates measured robot state 
        # using the provided measurements

        pass

    @abstractmethod
    def _solve(self) -> bool:

        pass
            
    @abstractmethod
    def _get_ndofs(self):

        pass

    @abstractmethod
    def _init_problem(self):

        # initialized horizon's TO problem

        pass
   
RHChild = TypeVar('RHChild', bound='RHController')