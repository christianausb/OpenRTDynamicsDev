//
//    Copyright (C) 2010, 2011, 2012, 2013  Christian Klauer
//
//    This file is part of OpenRTDynamics, the Real Time Dynamics Framework
//
//    OpenRTDynamics is free software: you can redistribute it and/or modify
//    it under the terms of the GNU Lesser General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    OpenRTDynamics is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU Lesser General Public License for more details.
//
//    You should have received a copy of the GNU Lesser General Public License
//    along with OpenRTDynamics.  If not, see <http://www.gnu.org/licenses/>.
//



//
// Example for varying sampling times
//
// This is an example for running simulations in a thread with non-constant
// sampling time. Using the "ld_synctimer" the interval time to the next
// simulation step can be specified by an input sigal to this block.
// Currently the "ld_synctimer" only works for nested simulations.
//
// Run with 
//
// $ sh ReadSensors.sh
//


thispath = get_absolute_file_path('ReadSensors.sce');
cd(thispath);


z = poly(0,'z');

T_a = 0.1;

//
// Set up simulation schematic
//




exec('../scilab_loader.sce');


function [sim, outlist, userdata] = run_thread_fn(sim, inlist, userdata)
  // This will run in a thread
  defaultevents = 0;
    

  //
  // Define non-constant sample times
  //

  [sim,Stop] = ld_play_simple(sim, events, r=[ zeros(100, 1); 1  ]);

  // Read Sensors
  ConfigStruct.rateAcc = 100; ConfigStruct.rateGyro = 100; ConfigStruct.rateMagn = 100; ConfigStruct.rateGPS = 0;

  [sim, out, SensorID] = ld_AndroidSensors(sim, events, in=Stop, ConfigStruct); // This synchronises the simulation
  [sim] = ld_printf(sim, events, out, "Sensors: ", 10);


  // output of schematic
  outlist = list();
endfunction




// This is the main top level schematic
function [sim, outlist] = schematic_fn(sim, inlist)  
  events = 0;

        [sim, zero] = ld_const(sim, events, 0);

//        [sim, startcalc] = ld_initimpuls(sim, events); // triggers your computation only once
        [sim, startcalc] = ld_const(sim, events, 1); // triggers your computation during each time step


        // Create a non-RT thread
        ThreadPrioStruct.prio1=ORTD.ORTD_RT_NORMALTASK; // or  ORTD.ORTD_RT_NORMALTASK
        ThreadPrioStruct.prio2=0; // for ORTD.ORTD_RT_REALTIMETASK: 1-99 as described in   man sched_setscheduler
        ThreadPrioStruct.cpu = -1; // -1 means dynamically assign CPU

        [sim, outlist] = ld_async_simulation(sim, events, ...
                              inlist=list(), ...
                              insizes=[], outsizes=[], ...
                              intypes=[], outtypes=[], ...
                              nested_fn = run_thread_fn, ...
                              TriggerSignal=startcalc, name="Thread1", ...
                              ThreadPrioStruct, userdata=list() );
       


  // output of schematic
  outlist = list();
endfunction



  
//
// Set-up
//

// defile events
defaultevents = [0]; // main event

// set-up schematic by calling the user defined function "schematic_fn"
insizes = []; outsizes=[];
[sim_container_irpar, sim]=libdyn_setup_schematic(schematic_fn, insizes, outsizes);



//
// Save the schematic to disk (possibly with other ones or other irpar elements)
//

parlist = new_irparam_set();

// pack simulations into irpar container with id = 901
parlist = new_irparam_container(parlist, sim_container_irpar, 901);

// irparam set is complete, convert to vectors
par = combine_irparam(parlist);

// save vectors to a file
save_irparam(par, 'ReadSensors.ipar', 'ReadSensors.rpar');

// clear
par.ipar = [];
par.rpar = [];





