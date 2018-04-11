// function [sim, out] = ld_AndroidSensors(sim, events, in, rate) // PARSEDOCU_BLOCK
// // %PURPOSE: Read out Android Sensors and synchronise the simulation to them
// //
// // Special: SYNC_BLOCK (use only one block of this type in an asynchronous running sub-simulation)
// //
// // out - vector of size 10 containing the sensor values
// // in - when in becomes one, the synchronisation loop is interrupted
// // 
// 
// 
//  btype = 15500 + 0; //
//  [sim,blk] = libdyn_new_block(sim, events, btype, ipar=[ 0  ], rpar=[ rate ], ...
//                   insizes=[1], outsizes=[10], ...
//                   intypes=[ORTD.DATATYPE_FLOAT], outtypes=[ORTD.DATATYPE_FLOAT]  );
// 
//  [sim,blk] = libdyn_conn_equation(sim, blk, list(in) );
//  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
// endfunction
// 
// 
// 


function [sim, out, SensorID] = ld_AndroidSensors(sim, events, in, ConfigStruct) // PARSEDOCU_BLOCK
// %PURPOSE: Read out Android Sensors and synchronise the simulation to them
//
// Special: SYNC_BLOCK (use only one block of this type in an asynchronous running sub-simulation)
//
// out - vector of size 10 containing the sensor values
// in - when in becomes one, the synchronisation loop is interrupted
// SensorID - The ID of the sensor that send new data
// 
// Possible sensor ID's:
// 
//     ASENSOR_TYPE_ACCELEROMETER      = 1,
//     ASENSOR_TYPE_MAGNETIC_FIELD     = 2,
//     ASENSOR_TYPE_GYROSCOPE          = 4,
//     ASENSOR_TYPE_LIGHT              = 5,
//     ASENSOR_TYPE_PROXIMITY          = 8
// 
// EXPERIMENTAL
// 




  printf("Including Android sensor block\n");	


  try
    ConfigStruct.rateAcc;
  catch
    ConfigStruct.rateAcc = 0;
  end

  try
    ConfigStruct.rateGyro; 
  catch
    ConfigStruct.rateGyro = 0;
  end

  try
    ConfigStruct.rateMagn; 
  catch
    ConfigStruct.rateMagn = 0;
  end

  try
    ConfigStruct.rateGPS;
  catch
    ConfigStruct.rateGPS = 0;
  end


// introduce some parameters that are refered to by id's
// 
    // pack all parameters into a structure "parlist"
    parlist = new_irparam_set();
 
    parlist = new_irparam_elemet_ivec(parlist, [ ConfigStruct.rateAcc; ConfigStruct.rateGyro; ConfigStruct.rateMagn; ConfigStruct.rateGPS ] , 10); // id = 10
//    parlist = new_irparam_elemet_ivec(parlist, vec, 11); // vector of integers (double vectors are similar, replace ivec with rvec)
//    parlist = new_irparam_elemet_ivec(parlist, ascii(str), 12); // id = 12; A string parameter
// 
    p = combine_irparam(parlist); // convert to two vectors of integers and floating point values respectively



// Set-up the block parameters and I/O ports
  Uipar = [ p.ipar ];
  Urpar = [ p.rpar ];
  btype = 15500 + 0; // Reference to the block's type (computational function). Use the same id you are giving via the "libdyn_compfnlist_add" C-function

  insizes=[1]; // Input port sizes
  outsizes=[10, 1]; // Output port sizes
  dfeed=[1];  // for each output 0 (no df) or 1 (a direct feedthrough to one of the inputs)
  intypes=[ORTD.DATATYPE_FLOAT]; // datatype for each input port
  outtypes=[ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_INT32]; // datatype for each output port

  blocktype = 1; // 1-BLOCKTYPE_DYNAMIC (if block uses states), 2-BLOCKTYPE_STATIC (if there is only a static relationship between in- and output)

  // Create the block
  [sim, blk] = libdyn_CreateBlockAutoConfig(sim, events, btype, blocktype, Uipar, Urpar, insizes, outsizes, intypes, outtypes, dfeed);
  
  // connect the inputs
 [sim,blk] = libdyn_conn_equation(sim, blk, list(in) ); // connect in1 to port 0 and in2 to port 1

  // connect the ouputs
 [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
 [sim,SensorID] = libdyn_new_oport_hint(sim, blk, 1);   // 0th port


//     ASENSOR_TYPE_ACCELEROMETER      = 1,
//     ASENSOR_TYPE_MAGNETIC_FIELD     = 2,
//     ASENSOR_TYPE_GYROSCOPE          = 4,
//     ASENSOR_TYPE_LIGHT              = 5,
//     ASENSOR_TYPE_PROXIMITY          = 8

endfunction

function [sim, finished, outlist] = ld_AutoExperiment(sim, ev, inlist, insizes, outsizes, intypes, outtypes, ThreadPrioStruct, experiment_fn, whileComputing_fn, evaluation_fn, whileIdle_fn)  // PARSEDOCU_BLOCK
    //
    // %PURPOSE: Perform automatic calibration procedures
    // 
    // Automatically perform an experiment and the ongoing evaluation.
    // The computation required for the evaluation is performed in a the background
    // by means of a thread.
    // 
    // There are several callback functions that describe:
    // 
    // experiment_fn: The schematic for performing the experiment, e.g. collecting data
    // whileComputing_fn: The schematic that is activated during the computation is active_state
    // evaluation_fn: The schematic that performs the evaulation in a thread.
    //                One simulation step is performed here
    // whileIdle_fn:  The schematic that is active when the procedure finished.
    // 
    // 
    // 
    // 
    // The prototypes are (Hereby outlist and inlist are lists of the signals that are forwarded 
    // to the I/O of ld_AutoExperiment:
    // 
    // [sim, finished, outlist] = experiment_fn(sim, ev, inlist)
    // [sim, outlist] = whileComputing_fn(sim, ev, inlist)
    // [sim, CalibrationOk, userdata] = evaluation_fn(sim, userdata)
    // [sim, outlist] = whileIdle_fn(sim, ev, inlist)
    // 
    // NOTE: Not everything is finished by now
    // 


    function [sim, outlist, userdata] = evaluation_Thread(sim, inlist, userdata)

        [sim, CalibrationOk, userdata] = evaluation_fn(sim, userdata);

        outlist = list(CalibrationOk);
    endfunction

    function [sim, outlist, active_state, x_global_kp1, userdata] = experiment_sm(sim, inlist, x_global, state, statename, userdata)
        // This function is called multiple times: once for each state.
        // At runtime these are different nested simulations. Switching
        // between them is done, where each simulation represents a
        // certain state.

        ev = 0;
        printf("ld_AutoExperiment: defining state %s (#%d) ...\n", statename, state);


        // print out some state information
        //     [sim] = ld_printf(sim, ev, in=x_global, str="<cntrl_state "+statename+"> x_global", insize=1);


        // define different controllers here
        select state
        case 1 // state 1
            // The experiment
            [sim, finished, outlist, userdata] = experiment_fn(sim, ev, inlist, userdata);

            [sim, active_state] = ld_const(sim, ev, 0);  // by default: no state switch       
            [sim, active_state ] = ld_cond_overwrite(sim, ev, in=active_state, condition=finished, setto=2); // Go to state 2 when finished

        case 2 // state 2
            // run something while the computation is running
            [sim, outlist, userdata] = whileComputing_fn(sim, ev, inlist, userdata);

            // Create a thread for performing the computation in the background
            [sim, startcalc] = ld_initimpuls(sim, 0); // triggers the computation only once when entering this state
            [sim, outlist__, computation_finished] = ld_async_simulation(sim, 0, ...
            inlist=list(), ...
            insizes=[], outsizes=[1], ...
            intypes=[], outtypes=[ORTD.DATATYPE_INT32], ...
            nested_fn = evaluation_Thread, ...
            TriggerSignal=startcalc, name="Comp Thread", ...
            ThreadPrioStruct, userdata=list() );


            //
            CalibrationOk = outlist__(1);
            [sim,CalibrationOk_] = ld_Int32ToFloat(sim, 0, CalibrationOk);

            // 	  [sim] = ld_printf(sim, ev, in=computation_finished, str="computation_finished", insize=1);

            [sim, FinshedOk] = ld_and(sim, 0, list( CalibrationOk_, computation_finished ));

            // WHEN TO CHANGE THE STATE
            [sim, active_state] = ld_const(sim, ev, 0);  // by default: no state switch
            [sim, active_state ] = ld_cond_overwrite(sim, ev, in=active_state, condition=computation_finished, setto=3); // go to state 3 if        

        case 3 // state 3
            [sim, outlist, userdata] = whileIdle_fn(sim, ev, inlist, userdata);

            [sim, active_state] = ld_const(sim, ev, 0);  // by default: no state switch       
        end

        x_global_kp1 = x_global;    
    endfunction




    // set-up three states represented by three nested simulations
    [sim, outlist, x_global, active_state,userdata] = ld_statemachine(sim, ev=ev, ...
    inlist, ..
    insizes, outsizes, ... 
    intypes, outtypes, ...
    nested_fn=experiment_sm, Nstates=3, state_names_list=list("experiment", "evaluation", "finished"), ...
    inittial_state=1, x0_global=[1], userdata=list()  );


    finished = active_state;
endfunction



function [sim, finished, outlist, userdata] = ld_AutoExperiment2(sim, ev, inlist, insizes, outsizes, intypes, outtypes, ThreadPrioStruct, experiment_fn, whileComputing_fn, evaluation_fn, whileIdle_fn, userdata)  // PARSEDOCU_BLOCK
    //
    // Automatically perform an experiment and the ongoing evaluation.
    // The computation required for the evaluation is performed in a the background
    // by means of a thread.
    // 
    // There are several callback functions that describe:
    // 
    // experiment_fn: The schematic for performing the experiment, e.g. collecting data
    // whileComputing_fn: The schematic that is activated during the computation is active_state
    // evaluation_fn: The schematic that performs the evaulation in a thread.
    //                One simulation step is performed here
    // whileIdle_fn:  The schematic that is active when the procedure finished.
    // 
    // 
    // 
    // 
    // The prototypes are (Hereby outlist and inlist are lists of the signals that are forwarded 
    // to the I/O of ld_AutoExperiment:
    // 
    // [sim, finished, outlist] = experiment_fn(sim, ev, inlist)
    // [sim, outlist] = whileComputing_fn(sim, ev, inlist)
    // [sim, CalibrationReturnVal, userdata] = evaluation_fn(sim, userdata)  NOTE changed this function
    // [sim, outlist] = whileIdle_fn(sim, ev, inlist)
    // 
    // NOTE: Not everything is finished by now
    // 
    // Rev 2: of ld_AutoExperiment: added userdata input/output, changed prototype of evaluation_fn
    // 


    function [sim, outlist, userdata] = evaluation_Thread(sim, inlist, userdata)
        [sim, CalibrationReturnVal, userdata] = evaluation_fn(sim, userdata);
        
         // TODO COMMENT
//        [sim] = ld_printfstderr(sim, ev, in=CalibrationReturnVal, str="-------------- > Computation step finished -- CalibrationReturnVal", insize=1);        

        outlist = list(CalibrationReturnVal);
    endfunction

    function [sim, outlist, active_state, x_global_kp1, userdata] = experiment_sm(sim, inlist, x_global, state, statename, userdata)
        // This function is called multiple times: once for each state.
        // At runtime these are different nested simulations. Switching
        // between them is done, where each simulation represents a
        // certain state.



        ev = 0;
//        printf("ld_AutoExperiment: defining state %s (#%d) ...\n", statename, state);


        // print out some state information
        //     [sim] = ld_printf(sim, ev, in=x_global, str="<cntrl_state "+statename+"> x_global", insize=1);


        // define different controllers here
        select state
        case 1 // state 1
            // The experiment

            [sim, finished, outlist, userdata] = experiment_fn(sim, ev, inlist, userdata);

            [sim, active_state] = ld_const(sim, ev, 0);  // by default: no state switch       
            [sim, active_state ] = ld_cond_overwrite(sim, ev, in=active_state, condition=finished, setto=2); // Go to state 2 when finished

        case 2 // state 2


            // Create a thread for performing the computation in the background
            [sim, startcalc] = ld_initimpuls(sim, 0); // triggers the computation only once when entering this state
            [sim, outlist__, computation_finished, userdata] = ld_async_simulation(sim, 0, ...
            inlist=list(), ...
            insizes=[], outsizes=[1], ...
            intypes=[], outtypes=[ORTD.DATATYPE_INT32], ...
            nested_fn = evaluation_Thread, ...
            TriggerSignal=startcalc, name="Comp Thread", ...
            ThreadPrioStruct, userdata );


            //
            CalibrationReturnVal = outlist__(1);
            [sim, CalibrationReturnVal_] = ld_Int32ToFloat(sim, 0, CalibrationReturnVal);
            [sim, JumperVals_] = ld_jumper(sim, 0, in=CalibrationReturnVal_, steps=2);
            [sim, JumperVals ] = ld_demux(sim, 0, 2, JumperVals_);

            //
            [sim, NotFinished] = ld_not(sim, 0, computation_finished);

            //           [sim, NotCalibrationOk_ ] = ld_not(sim, 0, CalibrationOk_);
            // 
            


            //          [sim] = ld_printf(sim, ev, in=NotFinished, str="computation_Notfinished", insize=1);// 

            //           [sim, FinshedOk   ] = ld_and(sim, 0, list(    CalibrationOk_ , computation_finished ));
            //           [sim, FinshedButNotOk] = ld_and(sim, 0, list( NotCalibrationOk_ , computation_finished ));


            // run something while the computation is running
            [sim, outlist, HoldState, userdata] = whileComputing_fn(sim, ev, inlist, CalibrationReturnVal, computation_finished, userdata);

            // TODO COMMENT
//            [sim] = ld_printf(sim, ev, in=computation_finished, str="computation_finished", insize=1);
//            [sim] = ld_printf(sim, ev, in=HoldState, str="HoldState", insize=1);
//            [sim] = ld_printf(sim, ev, in=startcalc, str="startcalc", insize=1);
            

            // WHEN TO CHANGE THE STATE
            [sim, active_state] = ld_const(sim, ev, 0);  // by default: no state switch
            [sim, active_state ] = ld_cond_overwrite(sim, ev, in=active_state, condition=JumperVals(1)  , setto=1); // return val 0 --> redo the calibration
            [sim, active_state ] = ld_cond_overwrite(sim, ev, in=active_state, condition=JumperVals(2)  , setto=3); // return val 1 --> run state 3

            [sim, active_state ] = ld_cond_overwrite(sim, ev, in=active_state, condition=NotFinished, setto=0); // If the computation is not finsihed do nothing at all
            [sim, active_state ] = ld_cond_overwrite(sim, ev, in=active_state, condition=HoldState, setto=0); // If whileComputing_fn desires to hold this state do nothing at all







        case 3 // state 3
            [sim, outlist, userdata] = whileIdle_fn(sim, ev, inlist, userdata);

            [sim, active_state] = ld_const(sim, ev, 0);  // by default: no state switch       
        end

        x_global_kp1 = x_global;    
    endfunction



    //  pause;
    // set-up three states represented by three nested simulations
    [sim, outlist, x_global, active_state,userdata] = ld_statemachine(sim, ev=ev, ...
    inlist, ..
    insizes, outsizes, ... 
    intypes, outtypes, ...
    nested_fn=experiment_sm, Nstates=3, state_names_list=list("experiment", "evaluation", "finished"), ...
    inittial_state=1, x0_global=[1], userdata  );


    finished = active_state;
endfunction




function [sim, finished, outlist, userdata] = ld_AutoOnlineExch_dev(sim, ev, inlist, insizes, outsizes, intypes, outtypes, ThreadPrioStruct, CallbackFns, ident_str, userdata)  // PARSEDOCU_BLOCK
    // 
    // %PURPOSE: Automated definition/compilation and execution of ORTD-schemtics during runtime.
    // 
    // Automatically perform an experiment and an ongoing evaluation. Additionally,
    // during this evaluation a new ORTD-schematic may be compiled to replace a part
    // of the control system.
    // 
    // The signals in inlist will be forwarded to several callback functions for defining e.g.
    // the nested control system.
    // 
    // There must be several callback functions in a structure CallbackFns:
    // 
    // experiment:      The schematic for performing the experiment, e.g. collecting data. This function
    //                  to define such a schematic may be called during compilation as well as during 
    //                  runtime of the control system. The latter case is used to replace the experiment
    //                  controller with an online-generated replacement that may depend e.g. on previously 
    //                  collected calibration data.
    // whileComputing:  The schematic that is activated during the computation is active_state
    // PreScilabRun:    This ORTD-schematic is called for one time step in advance to the embedded Scilab-calculation
    // 
    // 
    // Note: This is a wrapper to ld_AutoOnlineExch_dev2 for backwards compatibility. Use ld_AutoOnlineExch_dev2 instead.
    // 

    // a wrapper to the new version
    param.scilab_path = "BUILDIN_PATH";
    [sim, finished, outlist, userdata] = ld_AutoOnlineExch_dev2(sim, ev, inlist, insizes, outsizes, intypes, outtypes, ThreadPrioStruct, CallbackFns, ident_str, userdata, param)

endfunction








function [sim, finished, outlist, userdata] = ld_AutoOnlineExch_dev2(sim, ev, inlist, insizes, outsizes, intypes, outtypes, ThreadPrioStruct, CallbackFns, ident_str, userdata, param)  // PARSEDOCU_BLOCK
    // 
    // %PURPOSE: Automated definition/compilation and execution of ORTD-schemtics during runtime.
    // 
    // Automatically perform an experiment and an ongoing evaluation. Additionally,
    // during this evaluation a new ORTD-schematic may be compiled to replace a part
    // of the control system.
    // 
    // The signals in inlist will be forwarded to several callback functions for defining e.g.
    // the nested control system.
    // 
    // There must be several callback functions in a structure CallbackFns:
    // 
    // experiment:      The schematic for performing the experiment, e.g. collecting data. This function
    //                  to define such a schematic may be called during compilation as well as during 
    //                  runtime of the control system. The latter case is used to replace the experiment
    //                  controller with an online-generated replacement that may depend e.g. on previously 
    //                  collected calibration data.
    //
    //                  The prototype for this callback function is:
    //
    //                    [sim, finished, outlist, userdata] = experiment(sim, ev, inlist, userdata, CalledOnline)
    //
    //
    //
    // whileComputing:  The schematic that is activated while the computation is active_state
    //
    //                    [sim, outlist, HoldState, userdata] = whileComputing_example(sim, ev, inlist, CalibrationReturnVal, computation_finished, par)
    //
    //
    // PreScilabRun:    This ORTD-schematic is called for one time step in advance to the embedded Scilab-calculation
    //
    //                    [sim, ToScilab, userdata] = PreScilabRun(sim, ev, par)
    //
    // ident_str        Unique string for identifications of the created instance
    // userdata         left to the user of this function to transfer data to the callback functions
    //
    //
    // param            Contains additional parameters:
    // 
    // param.scilab_path: String describing the Scilab executable to use. e.g. set to "BUILDIN_PATH"
    // 
    // Note: This is a temporary development version. The interface may slightly change. Rev 2
    // 


    // extract the callback functions
    experiment_user = CallbackFns.experiment;
    whileComputing_user = CallbackFns.whileComputing;
    PreScilabRun_user = CallbackFns.PreScilabRun;

    function [sim, outlist, userdata ] = ExperimentReplaceable(sim, inlist, par)
        //    
        //    The nested simulation contains two sub-simulations:
        //    
        //    1) A schematic, which commonly contains nothing and is switched to
        //       when the replacement is in progress (which may take some time)
        //
        //    2) The schematic, which actually contains the algorithm to execute
        //
        //    Here, the initial simulations are defined, which can then be 
        //    replaced during runtime
        //    


        ev = 0;

        cntrlN = par(1); // the number of the nested schematics (one of two) "1" means the 
        // dummy schematic which is activated while the 2nd "2" is exchanged during runtime
        userdata = par(2);

//        printf("Compiling replaceable N=%d\n", cntrlN);   

        insizes = userdata(1); intypes = userdata(2); 
        outsizes=userdata(3); outtypes=userdata(4);
        useruserdata = userdata(5);
        experiment_user = userdata(6);

        [sim,zero] = ld_const(sim, ev, 0);

        if (cntrlN == 1) then  // is this the schematic 2) ?
            // Define dummy outputs
            // NOTE: only ORTD.DATATYPE_FLOAT is supported by this
            outlist = list(); 
            for outsize=outsizes        
                [sim, dummyOut] = ld_constvec(sim, 0, zeros( outsize, 1 ) );
                outlist($+1) = dummyOut;

            end
            outlist($+1) = zero; // control output
        end

        if (cntrlN == 2) then  // is this the schematic 2) ?

            printf("Compiling schematic: experiment_user\n");

            // run the callback function for definition of the experiment controller
            //       disp(fun2string(experiment_user));
            [sim, finished, outlist, useruserdata] = experiment_user(sim, 0, inlist, useruserdata, CalledOnline);

            printf("Compiling schematic: done\n");

            // store the userdata of experiment_user
            userdata(5) = useruserdata;

            // add control output
            outlist($+1) = finished;
        end

    endfunction


    function [sim, finished, outlist, userdata] = experiment(sim, ev, inlist, userdata)
        // Do the experiment

        insizes = userdata(1); intypes = userdata(2); 
        outsizes=userdata(3); outtypes=userdata(4);
        useruserdata = userdata(5);
        experiment_user = userdata(6);
        ident_str = userdata(7);

        [sim,zero] = ld_const(sim, ev, 0);
        [sim,active_sim] = ld_const(sim, 0, 1);

        //
        // Here the experiment controller is nested such that it can be replaced online
        //

        // append control signal to the outputs
        outsizes__ = outsizes; outsizes__($+1) = 1;
        outtypes__ = outtypes; outtypes__($+1) = ORTD.DATATYPE_FLOAT;


        CalledOnline = %f;  // The experiment function is not called online when going through 
        dfeed = 1;
        [sim, outlist_42342, computation_finished, userdata] = ld_simnest2(sim, 0 , ...
        inlist, ...
        insizes, outsizes__, ...
        intypes, ...
        outtypes__, ...
        nested_fn=ExperimentReplaceable, Nsimulations=2, dfeed, ...
        asynchron_simsteps=0, switch_signal=active_sim, ...
        reset_trigger_signal=zero, userdata, ...
        ident_str+"_ReplaceableSimulation" );

        finished = outlist_42342($); // the additional output

        //       [sim] = ld_printf(sim, 0, finished, "experiment: finished? " , 1);


        outlist = list( outlist_42342(1:$-1) );  // cut the last entry
    endfunction

    function [sim, outlist, HoldState, userdata] = whileComputing(sim, ev, inlist, CalibrationReturnVal, computation_finished, userdata);
        //   function [sim, outlist, userdata] = whileComputing(sim, ev, inlist, userdata)
        insizes = userdata(1); intypes = userdata(2); 
        outsizes=userdata(3); outtypes=userdata(4);
        useruserdata = userdata(5);
        experiment_user = userdata(6);

        // Callback
        par.userdata = useruserdata;
        par.insizes = insizes;
        par.outsizes = outsizes;
        par.intypes = intypes;
        par.outtypes = outtypes;

        [sim, outlist, HoldState, userdata] = whileComputing_user(sim, ev, inlist, CalibrationReturnVal, computation_finished, par);

    endfunction

    function [sim, outlist, userdata] = whileIdle(sim, ev, inlist, userdata)

        insizes = userdata(1); intypes = userdata(2); 
        outsizes=userdata(3); outtypes=userdata(4);
        useruserdata = userdata(5);
        experiment_user = userdata(6);
        ident_str = userdata(7);
        //       // Do wait
        //       [sim, readI] = ld_const(sim, ev, 1); // start at index 1
        //       [sim, Calibration] = ld_read_global_memory(sim, ev, index=readI, ident_str="CalibrationResult", ...
        //                                                   datatype=ORTD.DATATYPE_FLOAT, ...
        //                                                   ElementsToRead=20);

        //       [sim] = ld_printf(sim, 0, Calibration, "The calibration result is ", 20);

        //       // TODO: Dummy output      
        //       [sim, out] = ld_const(sim, ev, 0);
        //       outlist=list(out);

        // Define dummy outputs
        // NOTE: only ORTD.DATATYPE_FLOAT is supported by this
        outlist = list(); 
        for outsize=outsizes        
            [sim, dummyOut] = ld_constvec(sim, 0, zeros( outsize, 1 ) );
            outlist($+1) = dummyOut;
        end
    endfunction

    function [sim, CalibrationReturnVal, userdata] = evaluation(sim, userdata)
        // This superblock will run the evaluation of the experiment in a thread.
        // The superblock describes a sub-simulation, whereby only one step is simulated
        // which is enough to call scilab one signle time

        ev = 0;

        insizes = userdata(1); intypes = userdata(2); 
        outsizes=userdata(3); outtypes=userdata(4);
        useruserdata = userdata(5);
        experiment_user = userdata(6);
        ident_str = userdata(7);


        // define a Scilab function that performs the calibration
        function [block]=scilab_comp_fn( block, flag )
            // This scilab function is called during run-time
            // NOTE: Please note that the variables defined outside this
            //       function are typically not available at run-time.
            //       This also holds true for self defined Scilab functions!

            select flag
            case 1 // output
                tic();

                // split the sensor data
                data = block.inptr(1);

                block.states.ExecCounter = block.states.ExecCounter + 1;

                // get userdata
                try 
                    userdata = block.states.userdata;
                catch
                    userdata.isInitialised = %f;
                end

                // userdata should contain the sensor data
                userdata.SchematicInfo = "This schemtic was compiled during runtime in iteration #" + string(block.states.ExecCounter);
                userdata.InputData = data;


//                printf("Parameters to this computational Scilab function:\n");
//                disp(cfpar);
//
                insizes = cfpar.insizes;
                outsizes = cfpar.outsizes;
                intypes = cfpar.intypes;
                outtypes = cfpar.outtypes;
                ident_str = cfpar.ident_str;


                // 	    insizes=[2]; outsizes=[1];
                // 	    intypes=[ORTD.DATATYPE_FLOAT]; outtypes=[ORTD.DATATYPE_FLOAT];
                //             ident_str = "AutoCalibDemo";

                // append control signal to the outputs
                outsizes__ = outsizes; outsizes__($+1) = 1;
                outtypes__ = outtypes; outtypes__($+1) = ORTD.DATATYPE_FLOAT;

                //             printf("Defining schematic: experiment_user using the following function:\n");
                //             disp(fun2string(experiment_user));


                // create a new ir-par Experiment.[i,r]par files
                CalledOnline = %t;  // The experiment function is called online because we are in embedded Scilab here
                N = 2;
                [par, userdata] = ld_simnest2_replacement( ...
                insizes, outsizes__, ...
                intypes, outtypes__, ...
                nested_fn=ExperimentReplaceable, ...
                userdata=list(insizes, intypes, outsizes, outtypes, userdata, experiment_user), N);

                block.states.userdata = userdata(5);

                //
//                printf("__ New userdata is\n"); disp(block.states.userdata);

                // save vectors to a file
                save_irparam(par, ident_str+'_ReplaceableSimulation.ipar', ident_str+'_ReplaceableSimulation.rpar');

                // Tell that everything went fine.
                compready = 1;
                CalibrationReturnVal = 0; // run the experiment again

                // pack
                outvec = zeros(20,1);

                outvec(1) = compready;
                outvec(2) = CalibrationReturnVal;

                // clear
                par.ipar = [];
                par.rpar = [];

                block.outptr(1) = outvec;

                ElapsedTime = toc();
                printf("Time to run the embedded Scilab Code %f sec. \n", ElapsedTime);

            case 4 // init
                // printf("Setting funcproc(0), which has been of value %d before\n", funcprot());
                //funcproc(0);
                block.states.ExecCounter = 0;

            case 5 // terminate
                // 	    printf("terminate\n");

            case 10 // configure

            end
        endfunction


        par.userdata = useruserdata;


        // unload schematic the experiment schematic
        [sim, two] = ld_const(sim, 0, 2);
        [sim, out] = ld_nested_exchffile(sim, 0, compresult=two, slot=two, ... 
        fname=ident_str+"_ReplaceableSimulation", ident_str+"_ReplaceableSimulation");
        
        // run callback
        [sim, ToScilab, useruserdata] = PreScilabRun_user(sim, ev, par);
        [ToScilab_Size, ToScilab_type] = ld_getSizesAndTypes(sim, 0, SignalList=list(ToScilab) );

        //
        // Embedded Scilab. Run the function scilab_comp_fn defined above for one time step to perform the calibration
        // that is implemented in Scilab.
        //

        par.include_scilab_fns = list(ExperimentReplaceable, "ExperimentReplaceable", experiment_user, "experiment_user");

        function str = vec2str(v)
            LF = char(10);  // line feed char
            str = "[";
            for i=1:(length(v)-1)
                str = str + string(v(i)) + "," + LF;
            end
            str = str + string( v($) ) + "];";
        endfunction

        LF = char(10);
        par.InitStr = "cfpar.insizes=" + vec2str(insizes) + LF + "cfpar.intypes=" + vec2str(intypes) + LF...
        + "cfpar.outsizes=" + vec2str(outsizes) + LF + "cfpar.outtypes=" + vec2str(outtypes) + LF ...
        + "cfpar.ident_str=''" + ident_str + "'' " + LF;

        //        par.InitStr = "";
        //        disp(par.InitStr);


//        par.scilab_path = "BUILDIN_PATH";
        par.scilab_path = param.scilab_path;
        [sim, Calibration] = ld_scilab4(sim, 0, in=ToScilab, invecsize=ToScilab_Size(1), outvecsize=20, ...
        comp_fn=scilab_comp_fn, ForwardVars=%f, par);

        // Print the results
//         [sim] = ld_printf(sim, 0, Calibration, "The from Scilab returned values are ", 20);

        // demux      
        [sim, one] = ld_const(sim, 0, 1);      [sim, two] = ld_const(sim, 0, 2);
        [sim, compready] = ld_extract_element(sim, 0, invec=Calibration, pointer=one, vecsize=20 );
        [sim, CalibrationReturnVal] = ld_extract_element(sim, 0, invec=Calibration, pointer=two, vecsize=20 );


        // replace schematic the experiment schematic
        [sim, exchslot] = ld_const(sim, 0, 2);
        [sim, out] = ld_nested_exchffile(sim, 0, compresult=compready, slot=exchslot, ... 
        fname=ident_str+"_ReplaceableSimulation", ident_str+"_ReplaceableSimulation");
        //       [sim] = ld_FlagProbe(sim, 0, in=out, str="ASYNC COMP", 1);


        // run callback
        //       [sim, useruserdata] = PostScilabCalc(sim, 0, Calibration, useruserdata);


        // 
        userdata(5) = useruserdata;

    endfunction




    // Userdata for experiment_user
    //   userdata = list("This schemtic was compiled in advance to the execution");
    userdata = [];
    userdata.SchemeticInfo = "This schemtic was compiled in advance to the execution";


    [sim, finished, outlist, userdata] = ld_AutoExperiment2(sim, ev, inlist, ...
    insizes, outsizes, ...
    intypes , outtypes, ...
    ThreadPrioStruct, experiment, whileComputing, evaluation, whileIdle, ...
    userdata=list(insizes, intypes, outsizes, outtypes, userdata, experiment_user, ident_str)  );

endfunction











// 
// ortd_checkpar types:
// 
//     'Signal' 
//     'SignalList' 
//     'SingleValue' 
//     'Vector'
//     'String'
// 
//  e.g.
// 
//   ortd_checkpar(sim, list('Signal', 'in', in) );
//   ortd_checkpar(sim, list('SingleValue', 'gain', gain) );
// 



// TODO: FIXME: this is implemented twice. Remove the block below
// function [sim,out] = ld_getsign(sim, events, in) // PARSEDOCU_BLOCK
//   btype = 60001 + 26;


// 
// More basic functions that could also added to libdyn.sci
// 

function [sizes, types] = ld_getSizesAndTypes(sim, ev, SignalList)
//
// Tries to obtain the vector size and type of each signal given through the "SignalList"
//

    N = length(SignalList);
    sizes = zeros(N,1);
    types = zeros(N,1);

    for i=1:length(SignalList)
  //disp(i);
  try
      Signal = SignalList(i); // FIXME: This will only work for blocks, not system inputs, feed or other objects
      portN = sim.objectlist(Signal.oid).outport;
      sizes(i) = sim.objectlist(Signal.highleveloid).outsizes(portN+1);
      types(i) = sim.objectlist(Signal.highleveloid).outtypes(portN+1);
  catch
    printf("ld_getSizes\n");   
    error("ld_getSizesAndTypes: You have a source in SignalList, that doesn''t provide information about the vector size and datatype.");;
  end
    end
endfunction


// 
// More basic functions that could also added to libdyn.sci or so
// 



// Creates a Block that solely creates a globally shared object
function [sim] =  libdyn_CreateSharedObjBlk(sim, btype, ObjectIdentifyer, Visibility, Uipar, Urpar)
  
  events = 0;
  insizes=[]; // Input port sizes
  outsizes=[]; // Output port sizes
  dfeed=[1];  // for each output 0 (no df) or 1 (a direct feedthrough to one of the inputs)
  intypes=[]; // datatype for each input port
  outtypes=[]; // datatype for each output port

  blocktype = 2; // 1-BLOCKTYPE_DYNAMIC (if block uses states), 2-BLOCKTYPE_STATIC (if there is only a static relationship between in- and output)


  // Create the block
   [sim, blk] = libdyn_CreateBlockAutoConfig(sim, events, btype, blocktype, Uipar, Urpar, insizes, outsizes, intypes, outtypes, dfeed, ObjectIdentifyer, Visibility);
//   [sim, blk] = libdyn_CreateBlockAutoConfig(sim, events, btype, blocktype, Uipar, Urpar, insizes, outsizes, intypes, outtypes, dfeed, 'test');

  // ensure the block is included in the simulation even without any I/O ports
 sim = libdyn_include_block(sim, blk);
  
  // end new fn (sim)
endfunction

// new version for this functoin
// V2
function [sim, blk] = libdyn_CreateBlockAutoConfig(sim, events, btype, blocktype, Uipar, Urpar, insizes, outsizes, intypes, outtypes, dfeed, varargin)
  [lhs,rhs]=argn(0);
// 
// Create a I/O Configuration for the block that can be read out by the libdyn_AutoConfigureBlock() - C function
// during block's configuration
// 

  // ObjectIdentifyer


  if length(insizes) ~= length(intypes) then
    error("length(insizes) ~= length(intypes)");
  end
  if length(outsizes) ~= length(outtypes) then
    error("length(outsizes) ~= length(outtypes)");
  end

  

  param = [blocktype];

  parlist = new_irparam_set();

   parlist = new_irparam_elemet_ivec(parlist, insizes, 10); 
   parlist = new_irparam_elemet_ivec(parlist, outsizes, 11); 
   parlist = new_irparam_elemet_ivec(parlist, intypes, 12); 
   parlist = new_irparam_elemet_ivec(parlist, outtypes, 13); 
   parlist = new_irparam_elemet_ivec(parlist, dfeed, 14); 
   parlist = new_irparam_elemet_ivec(parlist, param, 15); 

   parlist = new_irparam_elemet_ivec(parlist, Uipar, 20); 
   parlist = new_irparam_elemet_rvec(parlist, Urpar, 21); 

   rhs=argn(2);
   if ( rhs > 11 ) then
     if (rhs == 13) then
       ObjectIdentifyer = varargin(1);
       Visibility = varargin(2);
       printf("Defining a Shared Object %s Visibility is %d\n", ObjectIdentifyer, Visibility);
       parlist = new_irparam_elemet_ivec(parlist, ascii(ObjectIdentifyer), 30); 
       parlist = new_irparam_elemet_ivec(parlist, Visibility, 31);     
     end
     if (rhs == 12) then
       ObjectIdentifyer = varargin(1);
       printf("Accessing a Shared Object %s\n", ObjectIdentifyer);
       parlist = new_irparam_elemet_ivec(parlist, ascii(ObjectIdentifyer), 30);        
     end
   end

   
   blockparam = combine_irparam(parlist);

  [sim,blk] = libdyn_new_block(sim, events, btype, ipar=[ blockparam.ipar  ], rpar=[ blockparam.rpar ], ...
                  insizes, outsizes, ...
                  intypes, outtypes );
endfunction




// V1
// function [sim, blk] = libdyn_CreateBlockAutoConfig(sim, events, btype, blocktype, Uipar, Urpar, insizes, outsizes, intypes, outtypes, dfeed)
// // 
// // Create a I/O Configuration for the block that can be read out by the libdyn_AutoConfigureBlock() - C function
// // during block's configuration
// // 
// 
//   // ObjectIdentifyer
// 
//   if length(insizes) ~= length(intypes) then
//     error("length(insizes) ~= length(intypes)");
//   end
//   if length(outsizes) ~= length(outtypes) then
//     error("length(outsizes) ~= length(outtypes)");
//   end
// 
//   
// 
//   param = [blocktype];
// 
//   parlist = new_irparam_set();
// 
//    parlist = new_irparam_elemet_ivec(parlist, insizes, 10); 
//    parlist = new_irparam_elemet_ivec(parlist, outsizes, 11); 
//    parlist = new_irparam_elemet_ivec(parlist, intypes, 12); 
//    parlist = new_irparam_elemet_ivec(parlist, outtypes, 13); 
//    parlist = new_irparam_elemet_ivec(parlist, dfeed, 14); 
//    parlist = new_irparam_elemet_ivec(parlist, param, 15); 
// 
//    parlist = new_irparam_elemet_ivec(parlist, Uipar, 20); 
//    parlist = new_irparam_elemet_rvec(parlist, Urpar, 21); 
// 
//    rhs=argn(2);
//    if ( rhs > 11 ) then
//      if (rhs == 13) then
//        printf("Defining a Shared Object %s Visibility is %d\n", ObjectIdentifyer, Visibility);
//        parlist = new_irparam_elemet_ivec(parlist, ascii(ObjectIdentifyer), 30); 
//        parlist = new_irparam_elemet_ivec(parlist, Visibility, 31);     
//      end
//      if (rhs == 12) then
//        printf("Accessing a Shared Object %s\n", ObjectIdentifyer);
//        parlist = new_irparam_elemet_ivec(parlist, ascii(ObjectIdentifyer), 30);        
//      end
//    end
// 
//    
//    blockparam = combine_irparam(parlist);
// 
//   [sim,blk] = libdyn_new_block(sim, events, btype, ipar=[ blockparam.ipar  ], rpar=[ blockparam.rpar ], ...
//                   insizes, outsizes, ...
//                   intypes, outtypes );
// 
// endfunction
// 









// 
// Interfacing functions are placed in this place
// 

function [sim] = ld_file_save_machine2(sim, ev, inlist, cntrl, FileNamesList) // PARSEDOCU_BLOCK
//
// %PURPOSE: Start and stop saving of multiple data vectors to multiple files
// 
//   inlist list() of *+ - Data to write
//   cntrl * - if cntrl steps to 2 then saving is started; if it steps to 1 saving is stopped
//   FileNamesList list() of strings - Filenames for saving
// 
// Note: This function tries to automatically detect the vector size for each entry of inlist.
//       Howver, this does not work for all signal sources (blocks) at the moment.
//       If come accross such a situation, you're invited to notify the authors of ORTD.
// 
// Note: The implementation of this function is a superblock using state machines
//       and the ld_savefile block. If you're invited to take a look at its source for a nice
//       example on using state machines.
// 
// 
// Example:
// 
//       TriggerSave = ...
// 
//       SaveSignals=list();        FileNamesList=list();
//       SaveSignals($+1) = Signal1;      FileNamesList($+1) = "measurements/Signal1.dat";
//       SaveSignals($+1) = Signal2;      FileNamesList($+1) = "measurements/Signal2.dat";
// 
//       [sim] = ld_file_save_machine2(sim, ev, ...
//                          inlist=SaveSignals, ...
//                          cntrl=TriggerSave, FileNamesList          );
// 

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'cntrl', cntrl) );
  ortd_checkpar(sim, list('SignalList', 'inlist', inlist) );
end


    function [sim, outlist, active_state, x_global_kp1, userdata] = state_mainfn(sim, inlist, x_global, state, statename, userdata)
      // This function is called multiple times -- once for each state.
      // At runtime, these are three different nested simulations. Switching
      // between them represents state changing, thus each simulation 
      // represents a certain state.
      
//       printf("defining savemachine state %s (#%d) ... userdata(1)=%s\n", statename, state, userdata(1) );
      
      // define names for the first event in the simulation
      ev = 0; events = ev;

      DataLength = userdata(1);      NumPorts = userdata(2);    FileNames = userdata(3);
      // 

      switch = inlist(NumPorts+1);  [sim, switch] = ld_gain(sim, ev, switch, 1);


      // demultiplex x_global
      [sim, x_global] = ld_demux(sim, events, vecsize=1, invec=x_global);


      // The signals "active_state" is used to indicate state switching: A value > 0 means the 
      // the state enumed by "active_state" shall be activated in the next time step.
      // A value less or equal to zero causes the statemachine to stay in its currently active
      // state

      select state
  case 1 // state 1
    active_state = switch;
//    [sim] = ld_printf(sim, ev, in=dataToSave, str="Pauseing Save", insize=DataLen);

  case 2 // state 2
      for i=1:NumPorts  // for each port a write to file block
        dataToSave = inlist(i);
      [sim] = ld_savefile(sim, ev, FileNames(i), source=dataToSave, vlen=DataLength(i) );
     // [sim] = ld_printf(sim, ev, in=dataToSave, str="Saveing port "+string(i)+" ", insize=DataLength(i) );
      end

    active_state = switch;
      end

      // multiplex the new global states
      [sim, x_global_kp1] = ld_mux(sim, ev, vecsize=1, inlist=x_global);
      
      // the user defined output signals of this nested simulation
      outlist = list();
  endfunction



//  if length(insizes) ~= length(intypes) then
//    error("ld_file_save_machine2: length(insizes) ~= length(intypes)");
//  end
//  if length(insizes) ~= length(inlist) then
//    error("ld_file_save_machine2: length(insizes) ~= length(inlist)");
//  end
  if length(inlist) ~= length(FileNamesList) then
    error("ld_file_save_machine2: length(insizes) ~= length(FileNamesList)");
  end


//  DataLength = insizes;
  NumPorts = length(inlist);


//   [sim] = ld_printf(sim, ev, cntrl, "cntrl", 1);

  [sim, cntrl] = ld_detect_step_event(sim, ev, in=cntrl, eps=0.2);

//   [sim] = ld_printf(sim, ev, cntrl, "Dcntrl", 1);

  Cinlist = list();
  for i=1:NumPorts
    Cinlist(i) = inlist(i);
  end
  Cinlist(NumPorts+1) = cntrl;

  // get the types and the sizes of the given signals
  [sizes, types] = ld_getSizesAndTypes(sim, ev, SignalList=Cinlist);

  DataLength = sizes;

  // set-up two states represented by two nested simulations
  [sim, outlist, x_global, active_state,userdata] = ld_statemachine(sim, ev=0, ...
      inlist=Cinlist, ..
      insizes=[sizes(:)' ], outsizes=[], ... 
      intypes=[types(:)' ], outtypes=[], ...
      nested_fn=state_mainfn, Nstates=2, state_names_list=list("pause", "save"), ...
      inittial_state=1, x0_global=[1], userdata=list(DataLength, NumPorts, FileNamesList)  );


endfunction


function [sim] = ld_MultiFileSave(sim, ev, inlist, cntrl, FileNamesList) // PARSEDOCU_BLOCK
//
// %PURPOSE: Start and stop saving of multiple data vectors to multiple files
// 
//   inlist list() of *+ - Data to write
//   cntrl * - if cntrl steps to 2 then saving is started; if it steps to 1 saving is stopped
//   FileNamesList list() of strings - Filenames for saving
// 
// Note: This function tries to automatically detect the vector size for each entry of inlist.
//       Howver, this does not work for all signal sources (blocks) at the moment.
//       If come accross such a situation, you're invited to notify the authors of ORTD.
// 
// Note: cntrl does not have an effect by now
// 
// Example:
// 
//       TriggerSave = ...
// 
//       SaveSignals=list();        FileNamesList=list();
//       SaveSignals($+1) = Signal1;      FileNamesList($+1) = "measurements/Signal1.dat";
//       SaveSignals($+1) = Signal2;      FileNamesList($+1) = "measurements/Signal2.dat";
// 
//       [sim] = ld_MultiFileSave(sim, 0, ...
//                          inlist=SaveSignals, ...
//                          cntrl=TriggerSave, FileNamesList          );
// 

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'cntrl', cntrl) );
  ortd_checkpar(sim, list('SignalList', 'inlist', inlist) );
end




  if length(inlist) ~= length(FileNamesList) then
    error("ld_file_save_machine2: length(insizes) ~= length(FileNamesList)");
  end


  NumPorts = length(inlist);

  // get the types and the sizes of the given signals
  [sizes, types] = ld_getSizesAndTypes(sim, 0, SignalList=inlist);
  
  for i=1:NumPorts  // for each port a write to file block

       dataToSave = inlist(i);
      [sim] = ld_savefile(sim, 0, FileNamesList(i), source=dataToSave, vlen=sizes(i) );

  end
  
 
endfunction



function [sim] = ld_savefile(sim, events, fname, source, vlen) // PARSEDOCU_BLOCK
//
// %PURPOSE: Quick and easy dumping of signals to files
// 
// fname - string of the filename
// source *+ - source signal
// vlen - vector size of signal
// 


if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'source', source) );
  ortd_checkpar(sim, list('SingleValue', 'vlen', vlen) );
  ortd_checkpar(sim, list('String', 'fname', fname) );
end

  [inp] = libdyn_extrakt_obj( source ); // compatibility

  autostart = 1;
  maxlen = 0
  fname = ascii(fname);

  btype = 130;

  [sim,blk] = libdyn_new_block(sim, events, btype, [maxlen, autostart, vlen, length(fname), fname(:)'], [],  ...
          insizes=[ vlen ], outsizes=[], ...
          intypes=[ ORTD.DATATYPE_FLOAT ], outtypes=[]  );
  
  [sim,save_] = libdyn_conn_equation(sim, blk, list(source) );
endfunction


function [sim, out] = ld_switch2to1(sim, events, cntrl, in1, in2) // PARSEDOCU_BLOCK
//
// %PURPOSE: A 2 to 1 switching Block
//
// cntr * - control input
// in1 *
// in2 *
// out * - output
//
// if cntrl > (greather than) 0 : out = in1
// if cntrl < (smaller than) 0 : out = in2
//

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'cntrl', cntrl) );
  ortd_checkpar(sim, list('Signal', 'in1', in1) );
  ortd_checkpar(sim, list('Signal', 'in2', in2) );
end


  btype = 60001;
  [sim,blk] = libdyn_new_block(sim, events, btype, [], [], ...
                   insizes=[1, 1, 1], outsizes=[1], ...
                   intypes=[ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_FLOAT], ...
                   outtypes=[ORTD.DATATYPE_FLOAT]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list(cntrl, in1, in2) );
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction





function [sim, outlist] = ld_demuxInt32(sim, events, vecsize, invec) // PARSEDOCU_BLOCK
//
// %PURPOSE: Demultiplexer int 32
//
// invec * - input vector signal to be split up
// outlist *LIST - list() of output signals
//
//
// Splits the input vector signal "invec" of size "vecsize" up into 
//
// outlist(1)
// outlist(2)
//  ....
//    

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'invec', invec) );
  ortd_checkpar(sim, list('SingleValue', 'vecsize', vecsize) );
end

  btype = 60001 + 37;  
  ipar = [vecsize, 0]; rpar = [];
  [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
                       insizes=[vecsize], outsizes=[ones(vecsize,1)], ...
                       intypes=[ORTD.DATATYPE_INT32], outtypes=[ORTD.DATATYPE_INT32*ones(vecsize,1)]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list(invec) );

  // connect each outport
  outlist = list();
  for i = 1:vecsize
    [sim,out] = libdyn_new_oport_hint(sim, blk, i-1);   // ith port
    outlist(i) = out;
  end
endfunction


function [sim, outlist] = ld_demux(sim, events, vecsize, invec) // PARSEDOCU_BLOCK
//
// %PURPOSE: Demultiplexer
//
// invec * - input vector signal to be split up
// outlist *LIST - list() of output signals
//
//
// Splits the input vector signal "invec" of size "vecsize" up into 
//
// outlist(1)
// outlist(2)
//  ....
//    

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'invec', invec) );
  ortd_checkpar(sim, list('SingleValue', 'vecsize', vecsize) );
end

  btype = 60001 + 1;  
  ipar = [vecsize, 0]; rpar = [];
  [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
                       insizes=[vecsize], outsizes=[ones(vecsize,1)], ...
                       intypes=[ORTD.DATATYPE_FLOAT], outtypes=[ORTD.DATATYPE_FLOAT*ones(vecsize,1)]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list(invec) );

  // connect each outport
  outlist = list();
  for i = 1:vecsize
    [sim,out] = libdyn_new_oport_hint(sim, blk, i-1);   // ith port
    outlist(i) = out;
  end
endfunction


function [sim, out] = ld_mux(sim, events, vecsize, inlist) // PARSEDOCU_BLOCK
//    
// %PURPOSE: Multiplexer
//
// inlist *LIST - list() of input signals of size 1
// out *+ - output vector signal
// 
// 
// combines inlist(1), inlist(2), ...    
// to a vector signal "out" of size "vecsize", whereby each inlist(i) is of size 1
//    

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('SignalList', 'inlist', inlist) );
  ortd_checkpar(sim, list('SingleValue', 'vecsize', vecsize) );
end

  btype = 60001 + 2;  
  ipar = [vecsize; 0]; rpar = [];

  if (length(inlist) ~= vecsize) then
    printf("Incorect number of input ports to ld_mux. %d != %d\n", length(inlist), vecsize );
    error(".");
  end

  [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
                                     insizes=[ones(1,vecsize)], outsizes=[vecsize], ...
                                     intypes=[ ORTD.DATATYPE_FLOAT*ones(1,vecsize) ], outtypes=[ORTD.DATATYPE_FLOAT] );
 
  // libdyn_conn_equation connects multiple input signals to blocks
  [sim,blk] = libdyn_conn_equation(sim, blk, inlist );

  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

function [sim, out] = ld_muxInt32(sim, events, vecsize, inlist) // PARSEDOCU_BLOCK
//    
// %PURPOSE: Multiplexer int 32
//
// inlist *LIST - list() of input signals of size 1
// out *+ - output vector signal
// 
// 
// combines inlist(1), inlist(2), ...    
// to a vector signal "out" of size "vecsize", whereby each inlist(i) is of size 1
//    

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('SignalList', 'inlist', inlist) );
  ortd_checkpar(sim, list('SingleValue', 'vecsize', vecsize) );
end

  btype = 60001 + 38;  
  ipar = [vecsize; 0]; rpar = [];

  if (length(inlist) ~= vecsize) then
    printf("Incorect number of input ports to ld_mux. %d != %d\n", length(inlist), vecsize );
    error(".");
  end

  [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
                                     insizes=[ones(1,vecsize)], outsizes=[vecsize], ...
                                     intypes=[ ORTD.DATATYPE_INT32*ones(1,vecsize) ], outtypes=[ORTD.DATATYPE_INT32] );
 
  // libdyn_conn_equation connects multiple input signals to blocks
  [sim,blk] = libdyn_conn_equation(sim, blk, inlist );

  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction


function [sim, out] = ld_hysteresis(sim, events, in, switch_on_level, switch_off_level, initial_state, onout, offout) // PARSEDOCU_BLOCK
// %PURPOSE: hysteresis block
//
// in * - input
// out * -output
// 
// switches out between onout and offout
// initial state is either -1 (off) or 1 (on)
//
//

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'in', in) );
  ortd_checkpar(sim, list('SingleValue', 'switch_off_level', switch_off_level) );
  ortd_checkpar(sim, list('SingleValue', 'switch_on_level', switch_on_level) );
  ortd_checkpar(sim, list('SingleValue', 'onout', onout) );
  ortd_checkpar(sim, list('SingleValue', 'offout', offout) );
end

  if (switch_off_level > switch_on_level) then
    error("ld_hysteresis: setting switch_off_level > switch_on_level makes no sense\n");
  end

  btype = 60001 + 3;
  [sim,blk] = libdyn_new_block(sim, events, btype, [initial_state], [ switch_on_level, switch_off_level, onout, offout] , ...
                   insizes=[1], outsizes=[1], ...
                   intypes=[ORTD.DATATYPE_FLOAT], outtypes=[ORTD.DATATYPE_FLOAT]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list(in) );
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction


function [sim, out] = ld_modcounter(sim, events, in, initial_count, mod) // PARSEDOCU_BLOCK
// %PURPOSE: Modulo Counter - Block
//
// in * - input
// out * -output
// 
// A counter that increases its value for each timestep for which in > 0 is true.
// if the counter value >= mod then it is reset to counter = initial_count
//
//

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'in', in) );
  ortd_checkpar(sim, list('SingleValue', 'initial_count', initial_count) );
  ortd_checkpar(sim, list('SingleValue', 'mod', mod) );
end

  if (mod < 0) then
    error("ld_modcounter: mod is less than zero\n");
  end

  btype = 60001 + 4;
  [sim,blk] = libdyn_new_block(sim, events, btype, [ initial_count, mod ], [  ],  ...
                   insizes=[1], outsizes=[1], ...
                   intypes=[ORTD.DATATYPE_FLOAT], outtypes=[ORTD.DATATYPE_FLOAT]   );

  [sim,blk] = libdyn_conn_equation(sim, blk, list(in) );
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction


function [sim, out] = ld_jumper(sim, events, in, steps) // PARSEDOCU_BLOCK
// %PURPOSE: jumper - block
//
// out *+ - vector of size steps
// in * - switching input
//
// The vector out always contains one "1", the rest is zero.
// The "1" moves to the right if in > 0. If the end is reached
// it "1" flips back to the left side
// 
// 

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'in', in) );
  ortd_checkpar(sim, list('SingleValue', 'steps', steps) );
end

  if (steps <= 0) then
    error("ld_jumper: steps must be greater than zero\n");
  end

  btype = 60001 + 5;
  [sim,blk] = libdyn_new_block(sim, events, btype, [ steps ], [  ], ...
                   insizes=[1], outsizes=[ steps ], ...
                   intypes=[ORTD.DATATYPE_FLOAT], outtypes=[ORTD.DATATYPE_FLOAT]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list(in) );
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction


function [sim, out] = ld_memory(sim, events, in, rememberin, initial_state) // PARSEDOCU_BLOCK
// %PURPOSE: memory - block
//
// in * - input
// rememberin * - 
// out * - output
// 
// If rememberin > 0 then
//   remember in, which is then feed to the output out until it is overwritten by a new value
//
// Please note that input ist applied to the output immediately (without a delay)
//
// initial output out = initial_state
// 

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'in', in) );
  ortd_checkpar(sim, list('Signal', 'rememberin', rememberin) );
  ortd_checkpar(sim, list('SingleValue', 'initial_state', initial_state) );
end


  memsize = length(initial_state);

  btype = 60001 + 6;
  [sim,blk] = libdyn_new_block(sim, events, btype, [ memsize  ], [ initial_state ],  ...
                   insizes=[memsize, 1], outsizes=[memsize], ...
                   intypes=[ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_FLOAT], outtypes=[ORTD.DATATYPE_FLOAT]   );

  [sim,blk] = libdyn_conn_equation(sim, blk, list(in, rememberin) );
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction


function [sim, out] = ld_abs(sim, events, in) // PARSEDOCU_BLOCK
// %PURPOSE: abs - block
//
// in * - input
// out * - output
// 
// out = abs(in)
// 


if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'in', in) );
end

  btype = 60001 + 7;
  [sim,blk] = libdyn_new_block(sim, events, btype, [  ], [  ], ...
                   insizes=[1], outsizes=[1], ...
                   intypes=[ORTD.DATATYPE_FLOAT], outtypes=[ORTD.DATATYPE_FLOAT]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list(in) );
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction


function [sim, out] = ld_extract_element(sim, events, invec, pointer, vecsize ) // PARSEDOCU_BLOCK
  //
  // %PURPOSE: Extract one element of a vector
  //
  // invec *+ - the input vector signal
  // pointer * - the index signal (indexing starts at 1)
  // vecsize - length of input vector
  // 
  // out = invec[pointer], the first element is at pointer = 1
  //

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'invec', invec) );
  ortd_checkpar(sim, list('Signal', 'pointer', pointer) );
  ortd_checkpar(sim, list('SingleValue', 'vecsize', vecsize) );
end

  btype = 60001 + 8;  
  ipar = [ vecsize, ORTD.DATATYPE_FLOAT ]; rpar = [];

  [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
                       insizes=[vecsize, 1], outsizes=[1], ...
                       intypes=[ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_FLOAT], outtypes=[ORTD.DATATYPE_FLOAT]  );

//   [sim,blk] = libdyn_new_blk_generic(sim, events, btype, ipar, rpar               );


  [sim,blk] = libdyn_conn_equation(sim, blk, list(invec, pointer) );

  // connect each outport
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // ith port
endfunction


function [sim, out] = ld_constvec(sim, events, vec) // PARSEDOCU_BLOCK
// 
// %PURPOSE: a constant vector
// 
// out *+ - the vector
// 


  btype = 60001 + 9;  
  ipar = [length(vec); 0]; rpar = [vec];

  [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
                   insizes=[], outsizes=[ length(vec) ], ...
                   intypes=[], outtypes=[ORTD.DATATYPE_FLOAT]  );
 
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction



function [sim, out] = ld_counter(sim, events, count, reset, resetto, initial) // PARSEDOCU_BLOCK
// 
// %PURPOSE: A resetable counter block
//
// count * - signal
// reset * - signal
// resetto * - signal
// initial - constant
// out * - output
// 
// increases out by count (out = out + count)
// 
// if reset > 0.5 then
//   out = resetto
//
// initially out is set to initial
// 
// 

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'count', count) );
  ortd_checkpar(sim, list('Signal', 'reset', reset) );  
  ortd_checkpar(sim, list('Signal', 'resetto', resetto) );
  ortd_checkpar(sim, list('SingleValue', 'initial', initial) );
end

  btype = 60001 + 10;
  ipar = [  ]; rpar = [ initial ];

  [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
                       insizes=[1,1,1], outsizes=[1], ...
                       intypes=[ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_FLOAT], ...
                       outtypes=[ORTD.DATATYPE_FLOAT]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list( count, reset, resetto ) );
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

function [sim, out] = ld_shift_register(sim, events, in, len) // FIXME TODO
// %PURPOSE: A shift register with access to the stored values
//
// in * - will be put to the first position in the register that was shifted before.
// out *+(len) - the whole memory
//    
// 
// 
// 

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'in', in) );
  ortd_checkpar(sim, list('SingleValue', 'len', len) );
end

  btype = 60001 + 11;
  ipar = [vecsize]; rpar = [];

  [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
                                     insizes=[ 1 ], outsizes=[len], ...
                                     intypes=[ ORTD.DATATYPE_FLOAT ], outtypes=[ORTD.DATATYPE_FLOAT] );
 
  // libdyn_conn_equation connects multiple input signals to blocks
  [sim,blk] = libdyn_conn_equation(sim, blk, list( in ) );

  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction






// Lookup table: Inputs between lowerin and upperin will be mapped linear to the indices of table
//               The corresponsing element of table will be the output
function [sim,bid] = libdyn_new_blk_lkup(sim, events, lowerin, upperin, table)
  btype = 120;
  [sim,bid] = libdyn_new_blk_generic(sim, events, btype, [length(table)], [ lowerin, upperin, table(:)' ]);
endfunction
function [sim, out] = ld_lookup(sim, events, u, lower_b, upper_b, table, interpolation) // PARSEDOCU_BLOCK
// %PURPOSE: Lookup table - block
//
// in * - input
// out * - output
// 
// 
// lower_b - smallest value of the input signal to map to the table
// upper_b - biggest value of the input signal to map to the table
// table - the table (Scilab vector)
// 
// Mapping is done in a linear way:
//   out = table( (in - lowerin) / (upperin - lowerin) )
// 
// interpolation = 0 : no interpolation
// interpolation = 1 : linear interpolation
// 
// 

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'u', u) );
  ortd_checkpar(sim, list('SingleValue', 'lower_b', lower_b) );
  ortd_checkpar(sim, list('SingleValue', 'upper_b', upper_b) );

  ortd_checkpar(sim, list('SingleValue', 'interpolation', interpolation) );
end

  btype = 60001 + 12;
  [sim,blk] = libdyn_new_block(sim, events, btype, [length(table), interpolation ], [ lower_b, upper_b, table(:)' ], ...
                   insizes=[1], outsizes=[1], ...
                   intypes=[ORTD.DATATYPE_FLOAT], outtypes=[ORTD.DATATYPE_FLOAT]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list(u) );
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction





function [sim, out] = ld_vector_lookup(sim, events, u, lower_b, upper_b, table, interpolation, vecsize) // PARSEDOCU_BLOCK
// %PURPOSE: Lookup table - block
//
// in *+(vecsize) - input
// out *+(vecsize) - output
// 
// 
// lower_b - smallest value of the input signal to map to the table
// upper_b - biggest value of the input signal to map to the table
// table - the table (Scilab vector)
// 
// Mapping is done in a linear way:
//   out = table( (in - lowerin) / (upperin - lowerin) )
// 
// interpolation = 0 : no interpolation
// interpolation = 1 : linear interpolation
// 
// 

  if ORTD.FASTCOMPILE==%f then
    ortd_checkpar(sim, list('Signal', 'u', u) );
    ortd_checkpar(sim, list('SingleValue', 'lower_b', lower_b) );
    ortd_checkpar(sim, list('SingleValue', 'upper_b', upper_b) );

    ortd_checkpar(sim, list('SingleValue', 'interpolation', interpolation) );
  end

  btype = 60001 + 72;
  [sim,blk] = libdyn_new_block(sim, events, btype, [length(table), interpolation, vecsize ], [ lower_b, upper_b, table(:)' ], ...
                   insizes=[ vecsize ], outsizes=[ vecsize ], ...
                   intypes=[ORTD.DATATYPE_FLOAT], outtypes=[ORTD.DATATYPE_FLOAT]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list(u) );
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction




function [sim, out] = ld_not(sim, events, in) // PARSEDOCU_BLOCK
// %PURPOSE: logic negation - block
//
// in * - input
// out * - output
// 
// out = 0, if in > 0.5  OR  out = 1, if in < 0.5
// 

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'in', in) );
end

  btype = 60001 + 13;
  [sim,blk] = libdyn_new_block(sim, events, btype, [  ], [  ], ...
                   insizes=[1], outsizes=[1], ...
                   intypes=[ORTD.DATATYPE_FLOAT], outtypes=[ORTD.DATATYPE_FLOAT]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list(in) );
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction


function [sim, out] = ld_or(sim, events, inlist) // PARSEDOCU_BLOCK
// %PURPOSE: logic or - block
//
// in *LIST - list() of inputs (for now the exactly two inputs are possible)
// out * - output
// 
// 
// 

//if ORTD.FASTCOMPILE==%f then
//  ortd_checkpar(sim, list('Signal', 'in', in) );
//end

  Nin=length(inlist);

  if (Nin ~= 2) then
    error("invalid number of inputs");
  end

  insizes=ones(1, Nin);
  intypes=ones(1, Nin) * ORTD.DATATYPE_FLOAT;

  btype = 60001 + 14;
  [sim,blk] = libdyn_new_block(sim, events, btype, ipar=[  ], rpar=[   ], ...
                   insizes, outsizes=[1], ...
                   intypes, outtypes=[ORTD.DATATYPE_FLOAT]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list( inlist(1), inlist(2) ) );
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

function [sim, out] = ld_iszero(sim, events, in, eps) // PARSEDOCU_BLOCK
//
// %PURPOSE: check if input is near zero
//
// in * - input
// out * - output
// 
// out = 1, if in between -eps and eps, othwewise out = 0
// 

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'in', in) );
  ortd_checkpar(sim, list('SingleValue', 'eps', eps) );
end

  btype = 60001 + 15;
  [sim,blk] = libdyn_new_block(sim, events, btype, ipar=[  ], rpar=[ eps ], ...
                   insizes=[1], outsizes=[1], ...
                   intypes=[ORTD.DATATYPE_FLOAT], outtypes=[ORTD.DATATYPE_FLOAT]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list(in) );
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

function [sim, out] = ld_limitedcounter(sim, events, count, reset, resetto, initial, lower_b, upper_b) // PARSEDOCU_BLOCK
// 
// %PURPOSE: A resetable, limited counter block
//
// count * - signal
// reset * - signal
// resetto * - signal
// initial - constant
// out * - output
// 
// increases out by count (out = out + count), but count is always between lower_b and upper_b
// 
// if reset > 0.5 then
//   out = resetto
//
// initially out is set to initial
// 
// 

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'count', count) );
  ortd_checkpar(sim, list('Signal', 'reset', reset) );
  ortd_checkpar(sim, list('Signal', 'resetto', resetto) );

  ortd_checkpar(sim, list('SingleValue', 'initial', initial) );
  ortd_checkpar(sim, list('SingleValue', 'lower_b', lower_b) );
  ortd_checkpar(sim, list('SingleValue', 'upper_b', upper_b) );
end

  if (lower_b > upper_b) then
    error("lower_b is greater than upper_b");
  end

  btype = 60001 + 16;
  ipar = [  ]; rpar = [ initial, lower_b, upper_b ];

  [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
                       insizes=[1,1,1], outsizes=[1], ...
                       intypes=[ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_FLOAT], ...
                       outtypes=[ORTD.DATATYPE_FLOAT]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list( count, reset, resetto ) );
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

function [sim, out] = ld_memorydel(sim, events, in, rememberin, initial_state) // PARSEDOCU_BLOCK
// %PURPOSE: delayed memory - block
//
// in * - input
// rememberin * - 
// out * - output
// 
// If rememberin > 0 then
//   remember in, which is then feed to the output "out" in the next time step until it is overwritten by a new value
//
// initial output out = initial_state
// 

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'in', in) );
  ortd_checkpar(sim, list('Signal', 'rememberin', rememberin) );
  ortd_checkpar(sim, list('SingleValue', 'initial_state', initial_state) );
end


  memsize = length(initial_state);

  btype = 60001 + 17;
  [sim,blk] = libdyn_new_block(sim, events, btype, [ memsize  ], [ initial_state ],  ...
                   insizes=[memsize, 1], outsizes=[memsize], ...
                   intypes=[ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_FLOAT], outtypes=[ORTD.DATATYPE_FLOAT]   );

  [sim,blk] = libdyn_conn_equation(sim, blk, list(in, rememberin) );
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

function [sim, out] = ld_steps(sim, events, activation_simsteps, values) // PARSEDOCU_BLOCK
//
// %PURPOSE: steps
//
// out * - output
// 
// 

//   ortd_checkpar(sim, list('Signal', 'in', in) );
//   ortd_checkpar(sim, list('SingleValue', 'eps', eps) );


  if (length(activation_simsteps) ~= length(values)-1) then
    error("length(activation_simsteps) != length(values)-1");
  end

  btype = 60001 + 18;
  [sim,blk] = libdyn_new_block(sim, events, btype, ipar=[ length(values), activation_simsteps ], rpar=[ values ], ...
                   insizes=[], outsizes=[1], ...
                   intypes=[], outtypes=[ORTD.DATATYPE_FLOAT]  );


  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

function [sim, out] = ld_cond_overwrite(sim, events, in, condition, setto) // PARSEDOCU_BLOCK
//
// %PURPOSE: conditional overwrite of the input signal's value
//
// out * - output
// in * - input to potentially overwrite
// condition * - condition signal
// 
// out = in, if condition < 0.5
// out = setto, otherwise
// 

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'in', in) );
  ortd_checkpar(sim, list('SingleValue', 'setto', setto) );
  ortd_checkpar(sim, list('Signal', 'condition', condition) );
end

  btype = 60001 + 19;
  [sim,blk] = libdyn_new_block(sim, events, btype, ipar=[ ], rpar=[ setto ], ...
                   insizes=[1,1], outsizes=[1], ...
                   intypes=[ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_FLOAT], outtypes=[ORTD.DATATYPE_FLOAT]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list(condition, in) );
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

function [sim, out] = ld_cond_overwrite2(sim, events, in, condition, setto) // PARSEDOCU_BLOCK
//
// %PURPOSE: conditional overwrite of the input signal's value
//
// out * - output (float)
// in * - input (float) to potentially overwrite
// condition * - condition signal (int32) -- in contrast to ld_cond_overwrite
// 
// out = in, if condition > 0
// out = setto, otherwise
// 

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'in', in) );
  ortd_checkpar(sim, list('SingleValue', 'setto', setto) );
  ortd_checkpar(sim, list('Signal', 'condition', condition) );
end

  btype = 60001 + 43;
  [sim,blk] = libdyn_new_block(sim, events, btype, ipar=[ ], rpar=[ setto ], ...
                   insizes=[1,1], outsizes=[1], ...
                   intypes=[ORTD.DATATYPE_INT32, ORTD.DATATYPE_FLOAT], outtypes=[ORTD.DATATYPE_FLOAT]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list(condition, in) );
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

function [sim, out] = ld_cond_overwriteInt32(sim, events, in, condition, setto) // PARSEDOCU_BLOCK
//
// %PURPOSE: conditional overwrite of the input signal's value
//
// out * - output (int32)
// in * - input (int32) to potentially overwrite
// condition * - condition signal (int32) -- in contrast to ld_cond_overwrite
// 
// out = in, if condition > 0
// out = setto, otherwise
// 

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'in', in) );
  ortd_checkpar(sim, list('SingleValue', 'setto', setto) );
  ortd_checkpar(sim, list('Signal', 'condition', condition) );
end

  btype = 60001 + 49;
  [sim,blk] = libdyn_new_block(sim, events, btype, ipar=[ setto ], rpar=[  ], ...
                   insizes=[1,1], outsizes=[1], ...
                   intypes=[ORTD.DATATYPE_INT32, ORTD.DATATYPE_INT32], outtypes=[ORTD.DATATYPE_INT32]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list(condition, in) );
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction




function [sim, out] = ld_ramp(sim, events, in_from, in_to, start, reset, ramp_duration) // NOT FINISHED
//
// %PURPOSE: Online configurable ramp block
//
// out * - output (from 0 to 1)
// start * - if > 0.5 the ramp starts
// reset * - if > 0.5 the blocks states are reset
// increase * - constant by which the output is increased for each time step
// 
// 

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'start', start) );
  ortd_checkpar(sim, list('Signal', 'reset', reset) );
  ortd_checkpar(sim, list('Signal', 'increase', increase) );
end

  btype = 60001 + 20;
  [sim,blk] = libdyn_new_block(sim, events, btype, ipar=[ ], rpar=[  ], ...
                   insizes=[1,1,1], outsizes=[1], ...
                   intypes=ORTD.DATATYPE_FLOAT*[1,1,1], outtypes=[ORTD.DATATYPE_FLOAT]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list(  start, reset, increase ) );
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

function [sim, out] = ld_and(sim, events, inlist) // PARSEDOCU_BLOCK
// %PURPOSE: logic and - block
//
// in *LIST - list() of inputs (for now the exactly two inputs are possible)
// out * - output
// 
// 

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('SignalList', 'inlist', inlist) );
end

  Nin=length(inlist);

  if (Nin ~= 2) then
    error("invalid number of inputs");
  end

  insizes=ones(1, Nin);
  intypes=ones(1, Nin) * ORTD.DATATYPE_FLOAT;

  btype = 60001 + 21;
  [sim,blk] = libdyn_new_block(sim, events, btype, ipar=[  ], rpar=[   ], ...
                   insizes, outsizes=[1], ...
                   intypes, outtypes=[ORTD.DATATYPE_FLOAT]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list( inlist(1), inlist(2) ) );
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction


function [sim, out] = ld_notInt32(sim, events, in) // PARSEDOCU_BLOCK
// %PURPOSE: logic negation - block
//
// in * - input
// out * - output
// 
// out = 0, if in >= 1  OR  out = 1, if in < 1
// 

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'in', in) );
end

  btype = 60001 + 46;
  [sim,blk] = libdyn_new_block(sim, events, btype, [  ], [  ], ...
                   insizes=[1], outsizes=[1], ...
                   intypes=[ORTD.DATATYPE_INT32], outtypes=[ORTD.DATATYPE_INT32]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list(in) );
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction


function [sim, out] = ld_andInt32(sim, events, inlist) // PARSEDOCU_BLOCK
// %PURPOSE: logic and - block
//
// in *LIST - list() of inputs (for now the exactly two inputs are possible)
// out * - output
// 
// 

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('SignalList', 'inlist', inlist) );
end

  Nin=length(inlist);

  if (Nin ~= 2) then
    error("invalid number of inputs");
  end

  insizes=ones(1, Nin);
  intypes=ones(1, Nin) * ORTD.DATATYPE_INT32;

  btype = 60001 + 44;
  [sim,blk] = libdyn_new_block(sim, events, btype, ipar=[  ], rpar=[   ], ...
                   insizes, outsizes=[1], ...
                   intypes, outtypes=[ORTD.DATATYPE_INT32]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list( inlist(1), inlist(2) ) );
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

function [sim, out] = ld_SetBitsInt32(sim, events, in, BitPattern, BitNrStart, NumBits) // PARSEDOCU_BLOCK
// %PURPOSE: Copy a bit-pattern to the input
//
// in *(INT32) - input
// BitPattern *(INT32)
// out *(INT32) - output
//
// BitNrStart - position in the input at which to start copying bits. Counting starts at zero
// NUmBits - number of bits to copy (NOTE; only ==1 works by now)
// 

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'inlist', in) );
  ortd_checkpar(sim, list('Signal', 'inlist', BitPattern) );

  ortd_checkpar(sim, list('SingleValue', 'inlist', BitNrStart) );
  ortd_checkpar(sim, list('SingleValue', 'inlist', NumBits) );
end




  btype = 60001 + 94;
  [sim,blk] = libdyn_new_block(sim, events, btype, ipar=[ BitNrStart,NumBits ], rpar=[   ], ...
                   insizes=[1,1], outsizes=[1], ...
                   intypes=[ORTD.DATATYPE_INT32,ORTD.DATATYPE_INT32], outtypes=[ORTD.DATATYPE_INT32]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list( in, BitPattern ) );
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction
function [sim, out] = ld_GetBitsInt32(sim, events, in, BitNrStart, NumBits) // PARSEDOCU_BLOCK
// %PURPOSE: Copy a range of bits from the input to the output
//
// in *(INT32) - input
// out *(INT32) - output
//
// BitNrStart - position in the input at which to start copying bits. Counting starts at zero
// NUmBits - number of bits to copy (NOTE; only ==1 works by now)
// 

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'inlist', in) );
 
  ortd_checkpar(sim, list('SingleValue', 'inlist', BitNrStart) );
  ortd_checkpar(sim, list('SingleValue', 'inlist', NumBits) );
end




  btype = 60001 + 95;
  [sim,blk] = libdyn_new_block(sim, events, btype, ipar=[ BitNrStart,NumBits ], rpar=[   ], ...
                   insizes=[1], outsizes=[1], ...
                   intypes=[ORTD.DATATYPE_INT32], outtypes=[ORTD.DATATYPE_INT32]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list( in ) );
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction





function [sim, out] = ld_orInt32(sim, events, inlist) // PARSEDOCU_BLOCK
// %PURPOSE: logic or - block
//
// in *LIST - list() of inputs (for now the exactly two inputs are possible)
// out * - output
// 
// 

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('SignalList', 'inlist', inlist) );
end

  Nin=length(inlist);

  if (Nin ~= 2) then
    error("invalid number of inputs");
  end

  insizes=ones(1, Nin);
  intypes=ones(1, Nin) * ORTD.DATATYPE_INT32;

  btype = 60001 + 45;
  [sim,blk] = libdyn_new_block(sim, events, btype, ipar=[  ], rpar=[   ], ...
                   insizes, outsizes=[1], ...
                   intypes, outtypes=[ORTD.DATATYPE_INT32]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list( inlist(1), inlist(2) ) );
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

function [sim, out] = ld_initimpuls(sim, events) // PARSEDOCU_BLOCK
//
// %PURPOSE: initial impuls
//
// out * - output
// 
// 


  btype = 60001 + 22;
  [sim,blk] = libdyn_new_block(sim, events, btype, ipar=[ ], rpar=[ ], ...
                   insizes=[], outsizes=[1], ...
                   intypes=[], outtypes=[ORTD.DATATYPE_FLOAT]  );


  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

function [sim] = ld_printfstderr(sim, events, in, str, insize) // PARSEDOCU_BLOCK
//
// %PURPOSE: Print data to stderr (the console)
//
// in *+(insize) - vectorial input signal
//
// str is a string that is printed followed by the signal vector in
// of size insize
//
  //[sim,blk] = libdyn_new_printf(sim, events, str, insize);

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'in', in) );
  ortd_checkpar(sim, list('String', 'str', str) );
  ortd_checkpar(sim, list('SingleValue', 'insize', insize) );
end

  btype = 60001 + 23;;
  str = ascii(str);
//   [sim,bid] = libdyn_new_blk_generic(sim, events, btype, [insize, length(str), str(:)'], []);

  [sim,blk] = libdyn_new_block(sim, events, btype, ipar=[ insize, length(str), str(:)' ], rpar=[ ], ...
                   insizes=[ insize ], outsizes=[], ...
                   intypes=[ ORTD.DATATYPE_FLOAT ], outtypes=[]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list(in) );
endfunction

function [sim] = ld_printfstderr2(sim, events, in, str, insize) // PARSEDOCU_BLOCK
//
// %PURPOSE: Print str to stderr (the console)
//
// in *+(insize) - vectorial input signal
//
// str is a string that is printed followed by the signal vector in
// of size insize
//
  //[sim,blk] = libdyn_new_printf(sim, events, str, insize);

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'in', in) );
  ortd_checkpar(sim, list('String', 'str', str) );
  ortd_checkpar(sim, list('SingleValue', 'insize', insize) );
end

  btype = 60001 + 36;;
  str = ascii(str);
//   [sim,bid] = libdyn_new_blk_generic(sim, events, btype, [insize, length(str), str(:)'], []);

  [sim,blk] = libdyn_new_block(sim, events, btype, ipar=[ insize, length(str), str(:)' ], rpar=[ ], ...
                   insizes=[ insize ], outsizes=[], ...
                   intypes=[ ORTD.DATATYPE_FLOAT ], outtypes=[]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list(in) );
endfunction

function [sim] = ld_printfbar(sim, events, in, str) // PARSEDOCU_BLOCK
//
// %PURPOSE: Print a bar (the console)
//
// in *+(1) - vectorial input signal
//
// str is a string that is printed followed by a bar whose length depends on in
//
  
if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'in', in) );
  ortd_checkpar(sim, list('String', 'str', str) );
end

  btype = 60001 + 29;
  str = ascii(str);
  insize = 1;
//   [sim,bid] = libdyn_new_blk_generic(sim, events, btype, [insize, length(str), str(:)'], []);

  [sim,blk] = libdyn_new_block(sim, events, btype, ipar=[ insize, length(str), str(:)' ], rpar=[ ], ...
                   insizes=[ insize ], outsizes=[], ...
                   intypes=[ ORTD.DATATYPE_FLOAT ], outtypes=[]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list(in) );
endfunction


function [sim, out] = ld_delay(sim, events, u, N) // PARSEDOCU_BLOCK
// %PURPOSE: delay - block
//
// in * - input
// out * - output
// 
// delay in by N steps
// 
// 

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'u', u) );
  ortd_checkpar(sim, list('SingleValue', 'N', N) );
end

  if length(N) ~= 1 then
    error("N is not a scalar\n");
  end

  if (N < 1) then
    error("invalid delay");
  end

  btype = 60001 + 24;
  [sim,blk] = libdyn_new_block(sim, events, btype, [ N ], [ ], ...
                   insizes=[1], outsizes=[1], ...
                   intypes=[ORTD.DATATYPE_FLOAT], outtypes=[ORTD.DATATYPE_FLOAT]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list(u) );
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction



function [sim, out] = ld_steps2(sim, events, activation_simsteps, values) // PARSEDOCU_BLOCK
//
// %PURPOSE: steps, counter is increased on event, which is different to ld_steps
//
// out * - output
// 
// 

  if (length(activation_simsteps) ~= length(values)-1) then
    error("length(activation_simsteps) != length(values)-1");
  end

  btype = 60001 + 25;
  [sim,blk] = libdyn_new_block(sim, events, btype, ipar=[ length(values), activation_simsteps ], rpar=[ values ], ...
                   insizes=[], outsizes=[1], ...
                   intypes=[], outtypes=[ORTD.DATATYPE_FLOAT]  );


  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction




// function [sim,out] = ld_getsign(sim, events, in) // PARSEDOCU_BLOCK
// //
// // %PURPOSE: return the sign of the input sigal
// // either 1 or -1
// //
// 
//   ortd_checkpar(sim, list('Signal', 'in', in) );
// 
// 
//   btype = 60001 + 26;
//   [sim,blk] = libdyn_new_block(sim, events, btype, [  ], [  ], ...
//                    insizes=[1], outsizes=[1], ...
//                    intypes=[ORTD.DATATYPE_FLOAT], outtypes=[ORTD.DATATYPE_FLOAT]  );
// 
//   [sim,blk] = libdyn_conn_equation(sim, blk, list(in) );
//   [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
// endfunction


function [sim, out] = ld_insert_element(sim, events, in, pointer, vecsize ) // PARSEDOCU_BLOCK
  //
  // %PURPOSE: Insert one element into a vector
  //
  // in *+ - the input element signal
  // pointer * - the index signal
  // vecsize - length of output vector
  // 
  // out[pointer] = in, the first element is at pointer = 1
  //

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'in', in) );
  ortd_checkpar(sim, list('Signal', 'pointer', pointer) );
  ortd_checkpar(sim, list('SingleValue', 'vecsize', vecsize) );
end

  btype = 60001 + 27; 
  ipar = [ vecsize, ORTD.DATATYPE_FLOAT ]; rpar = [];

  [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
                       insizes=[1, 1], outsizes=[vecsize], ...
                       intypes=[ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_FLOAT], outtypes=[ORTD.DATATYPE_FLOAT]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list(in, pointer) );

  // connect each outport
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // ith port
endfunction

function [sim] = ld_FlagProbe(sim, events, in, str, insize) // PARSEDOCU_BLOCK
//
// %PURPOSE: Print data and Flags (calc output, update states, reset states) to stderr (the console)
//
// in *+(insize) - vectorial input signal
//
// str is a string that is printed followed by the signal vector in
// of size insize
//

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'in', in) );
  ortd_checkpar(sim, list('String', 'str', str) );
  ortd_checkpar(sim, list('SingleValue', 'insize', insize) );
end

  //[sim,blk] = libdyn_new_printf(sim, events, str, insize);
  btype = 60001 + 28;
  str = ascii(str);
//   [sim,bid] = libdyn_new_blk_generic(sim, events, btype, [insize, length(str), str(:)'], []);

  [sim,blk] = libdyn_new_block(sim, events, btype, ipar=[ insize, length(str), str(:)' ], rpar=[ ], ...
                   insizes=[ insize ], outsizes=[], ...
                   intypes=[ ORTD.DATATYPE_FLOAT ], outtypes=[]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list(in) );
endfunction

function [sim,out] = ld_ceilInt32(sim, events, in) // PARSEDOCU_BLOCK
//
// %PURPOSE: ceil(in)
// 
// return value is of type ORTD.DATATYPE_INT32
// 

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'in', in) );
end

  btype = 60001 + 30;
  [sim,blk] = libdyn_new_block(sim, events, btype, [  ], [  ], ...
                   insizes=[1], outsizes=[1], ...
                   intypes=[ORTD.DATATYPE_FLOAT], outtypes=[ORTD.DATATYPE_INT32]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list(in) );
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

function [sim,out] = ld_Int32ToFloat(sim, events, in) // PARSEDOCU_BLOCK
//
// %PURPOSE: Convert int32 to double
// 
// ORTD.DATATYPE_INT32 --> ORTD.DATATYPE_FLOAT
// 

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'in', in) );
end

  btype = 60001 + 31;
  [sim,blk] = libdyn_new_block(sim, events, btype, [  ], [  ], ...
                   insizes=[1], outsizes=[1], ...
                   intypes=[ORTD.DATATYPE_INT32], outtypes=[ORTD.DATATYPE_FLOAT]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list(in) );
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

function [sim,out] = ld_vector_Int32ToFloat(sim, events, in, vecsize) // PARSEDOCU_BLOCK
//
// %PURPOSE: Convert int32 to double
// 
// ORTD.DATATYPE_INT32 --> ORTD.DATATYPE_FLOAT
// 

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'in', in) );
end

  btype = 60001 + 77;
  [sim,blk] = libdyn_new_block(sim, events, btype, [ vecsize ], [  ], ...
                   insizes=[ vecsize ], outsizes=[ vecsize ], ...
                   intypes=[ORTD.DATATYPE_INT32], outtypes=[ORTD.DATATYPE_FLOAT]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list(in) );
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

function [sim,out] = ld_floorInt32(sim, events, in) // PARSEDOCU_BLOCK
//
// %PURPOSE: ceil(in)
// 
// return value is of type ORTD.DATATYPE_INT32
// 

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'in', in) );
end

  btype = 60001 + 32;
  [sim,blk] = libdyn_new_block(sim, events, btype, [  ], [  ], ...
                   insizes=[1], outsizes=[1], ...
                   intypes=[ORTD.DATATYPE_FLOAT], outtypes=[ORTD.DATATYPE_INT32]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list(in) );
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

function [sim,out] = ld_vector_floorInt32(sim, events, in, vecsize) // PARSEDOCU_BLOCK
//
// %PURPOSE: ceil(in)
// 
// return value is of type ORTD.DATATYPE_INT32
// 

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'in', in) );
end

  btype = 60001 + 73;
  [sim,blk] = libdyn_new_block(sim, events, btype, [ vecsize ], [  ], ...
                   insizes=[ vecsize ], outsizes=[ vecsize ], ...
                   intypes=[ORTD.DATATYPE_FLOAT], outtypes=[ORTD.DATATYPE_INT32]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list(in) );
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

function [sim,out] = ld_roundInt32(sim, events, in) // PARSEDOCU_BLOCK
//
// %PURPOSE: ceil(in)
// 
// return value is of type ORTD.DATATYPE_INT32
// 

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'in', in) );
end

  btype = 60001 + 33;
  [sim,blk] = libdyn_new_block(sim, events, btype, [  ], [  ], ...
                   insizes=[1], outsizes=[1], ...
                   intypes=[ORTD.DATATYPE_FLOAT], outtypes=[ORTD.DATATYPE_INT32]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list(in) );
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

function [sim, out] = ld_constvecInt32(sim, events, vec) // PARSEDOCU_BLOCK
// 
// %PURPOSE: a constant vector of ORTD.DATATYPE_INT32
// 
// out *+ - the vector of int32
// 
  btype = 60001 + 34; 
  ipar = [length(vec); 0; vec]; rpar = [];

  [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
                   insizes=[], outsizes=[ length(vec) ], ...
                   intypes=[], outtypes=[ORTD.DATATYPE_INT32]  );
 
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction



function [sim,out] = ld_sumInt32(sim, events, in1, in2) // PARSEDOCU_BLOCK
//
// %PURPOSE: return sum of the input signals
// TODO
//

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'in1', in1) );
  ortd_checkpar(sim, list('Signal', 'in2', in2) );
end

  btype = 60001 + 35;
  [sim,blk] = libdyn_new_block(sim, events, btype, [  ], [  ], ...
                   insizes=[1,1], outsizes=[1], ...
                   intypes=[ORTD.DATATYPE_INT32, ORTD.DATATYPE_INT32], outtypes=[ORTD.DATATYPE_INT32]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list(in1, in2) );
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

function [sim,out] = ld_getsign(sim, events, in) // PARSEDOCU_BLOCK
//
// %PURPOSE: return - in
// TODO
//

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'in', in) );
end

  btype = 60001 + 37;
  [sim,blk] = libdyn_new_block(sim, events, btype, [  ], [  ], ...
                   insizes=[1], outsizes=[1], ...
                   intypes=[ORTD.DATATYPE_INT32], outtypes=[ORTD.DATATYPE_INT32]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list(in) );
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

function [sim,out] = ld_OneStepDelInt32(sim, events, in, init_state) // PARSEDOCU_BLOCK
//
// %PURPOSE: One time step delay for Int32
// TODO
//

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'in', in) );
  ortd_checkpar(sim, list('SingleValue', 'init_state', init_state) );
end

  btype = 60001 + 38;
  [sim,blk] = libdyn_new_block(sim, events, btype, [ init_state ], [  ], ...
                   insizes=[1], outsizes=[1], ...
                   intypes=[ORTD.DATATYPE_INT32], outtypes=[ORTD.DATATYPE_INT32]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list(in) );
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

function [sim,out] = ld_MulInt32(sim, ev, in1, in2) // PARSEDOCU_BLOCK
//
// %PURPOSE: return multiplication of the input signals
// TODO
//

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'in1', in1) );
  ortd_checkpar(sim, list('Signal', 'in2', in2) );
end

  btype = 60001 + 39;
  [sim,blk] = libdyn_new_block(sim, events, btype, [  ], [  ], ...
                   insizes=[1,1], outsizes=[1], ...
                   intypes=[ORTD.DATATYPE_INT32, ORTD.DATATYPE_INT32], outtypes=[ORTD.DATATYPE_INT32]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list(in1, in2) );
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

function [sim,out] = ld_DivInt32(sim, ev, num, den) // PARSEDOCU_BLOCK
//
// %PURPOSE: return num DIV den
// TODO. not implemented by now
//

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'num', num) );
  ortd_checkpar(sim, list('Signal', 'den', den) );
end

  btype = 60001 + 40;
  [sim,blk] = libdyn_new_block(sim, events, btype, [  ], [  ], ...
                   insizes=[1,1], outsizes=[1], ...
                   intypes=[ORTD.DATATYPE_INT32, ORTD.DATATYPE_INT32], outtypes=[ORTD.DATATYPE_INT32]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list(num, den) );
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

function [sim,out] = ld_ModInt32(sim, ev, num, den) // PARSEDOCU_BLOCK
//
// %PURPOSE: return num MODULO den
// TODO: not implemented by now
//

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'num', num) );
  ortd_checkpar(sim, list('Signal', 'den', den) );
end

  btype = 60001 + 41;
  [sim,blk] = libdyn_new_block(sim, events, btype, [  ], [  ], ...
                   insizes=[1,1], outsizes=[1], ...
                   intypes=[ORTD.DATATYPE_INT32, ORTD.DATATYPE_INT32], outtypes=[ORTD.DATATYPE_INT32]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list(num, den) );
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

function [sim,out] = ld_CompareEqInt32(sim, events, in, CompVal) // PARSEDOCU_BLOCK
//
// %PURPOSE: Compeare to CompVal (if equal)
// 
// in*, float
// out*, int32 - 0 if (in == CompVal); 1 if (in != CompVal);
//

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'in', in) );
  ortd_checkpar(sim, list('SingleValue', 'CompVal', CompVal) );
end

  btype = 60001 + 42;
  [sim,blk] = libdyn_new_block(sim, events, btype, [ CompVal ], [  ], ...
                   insizes=[1], outsizes=[1], ...
                   intypes=[ORTD.DATATYPE_INT32], outtypes=[ORTD.DATATYPE_INT32]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list(in) );
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

function [sim,out] = ld_CompareInt32(sim, events, in, Thr) // PARSEDOCU_BLOCK
//
// %PURPOSE: Compeare to Thr (if greater)
// 
// in*, float
// out*, int32 - 0 if (in > CompVal); 1 if (in != CompVal);
//

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'in', in) );
  ortd_checkpar(sim, list('SingleValue', 'Thr', Thr) );
end

  btype = 60001 + 47;
  [sim,blk] = libdyn_new_block(sim, events, btype, [ Thr ], [  ], ...
                   insizes=[1], outsizes=[1], ...
                   intypes=[ORTD.DATATYPE_INT32], outtypes=[ORTD.DATATYPE_INT32]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list(in) );
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction


function [sim,out] = ld_integratorInt32(sim, events, in) // PARSEDOCU_BLOCK
//
// %PURPOSE: Integrator on Int32
// 
// in*, int32
// out*, int32  out[k] = out[k-1] + in[k]
//

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'in', in) );
end

  btype = 60001 + 48;
  [sim,blk] = libdyn_new_block(sim, events, btype, [  ], [  ], ...
                   insizes=[1], outsizes=[1], ...
                   intypes=[ORTD.DATATYPE_INT32], outtypes=[ORTD.DATATYPE_INT32]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list(in) );
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction






















//   ortd_checkpar(sim, list('Signal', 'in', in) );
//   ortd_checkpar(sim, list('String', 'str', str) );
//   ortd_checkpar(sim, list('SingleValue', 'insize', insize) );




// 
//  Vector functions
// 

function [sim, out] = ld_vector_delay(sim, events, in, vecsize) // PARSEDOCU_BLOCK
// %PURPOSE: delay - block
//
// in * - input
// out * - output
// vecsize - size of vector in*
// 
// delay the hole vector in by one step
// 
// 

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'in', in) );
  ortd_checkpar(sim, list('SingleValue', 'vecsize', vecsize) );
end

  btype = 60001 + 65;
  [sim,blk] = libdyn_new_block(sim, events, btype, [ vecsize ], [ ], ...
                   insizes=[vecsize], outsizes=[vecsize], ...
                   intypes=[ORTD.DATATYPE_FLOAT], outtypes=[ORTD.DATATYPE_FLOAT]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list(in) );
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

function [sim, out] = ld_vector_delayInt32(sim, events, in, vecsize) // PARSEDOCU_BLOCK
// %PURPOSE: delay - block
//
// in * - input (int32)
// out * - output (int32)
// vecsize - size of vector in*
// 
// delay the hole vector in by one step
// 
// 

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'in', in) );
  ortd_checkpar(sim, list('SingleValue', 'vecsize', vecsize) );
end

  btype = 60001 + 84;
  [sim,blk] = libdyn_new_block(sim, events, btype, [ vecsize ], [ ], ...
                   insizes=[vecsize], outsizes=[vecsize], ...
                   intypes=[ORTD.DATATYPE_INT32], outtypes=[ORTD.DATATYPE_INT32]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list(in) );
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction


function [sim, out] = ld_vector_diff(sim, events, in, vecsize) // PARSEDOCU_BLOCK
//    
// %PURPOSE: Vector differentiation with respect to the index
// 
// in *+(vecsize) - vector signal of size "vecsize"
// out *+(vecsize-1) - vector signal of size "vecsize-1"
//
// Equivalent to Scilab 'diff' function
//    

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'in', in) );
  ortd_checkpar(sim, list('SingleValue', 'vecsize', vecsize) );
end

  btype = 60001 + 50; 
  ipar = [vecsize]; rpar = [];

  [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
                                     insizes=[vecsize], outsizes=[vecsize-1], ...
                                     intypes=[ ORTD.DATATYPE_FLOAT ], outtypes=[ORTD.DATATYPE_FLOAT] );
 
  // libdyn_conn_equation connects multiple input signals to blocks
  [sim,blk] = libdyn_conn_equation(sim, blk, list( in ) );

  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

function [sim, index] = ld_vector_findthr(sim, events, in, thr, greater, vecsize) // PARSEDOCU_BLOCK
// %PURPOSE: Find the index of the value in a vector that is grater than a given constant
//
// in *+(vecsize) - input
// thr * - threshold signal
// index * - output
// 
// find values greater than threshold "thr" in vector signal "in", when greater > 0
// find values less than threshold "thr" in vector signal "in", when greater =< 0
// 
//
//    
  btype = 60001 + 51; 
  ipar = [vecsize; greater]; rpar = [];

  [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
                                     insizes=[vecsize, 1], outsizes=[1], ...
                                     intypes=[ ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_FLOAT ], outtypes=[ORTD.DATATYPE_FLOAT] );
 
  // libdyn_conn_equation connects multiple input signals to blocks
  [sim,blk] = libdyn_conn_equation(sim, blk, list( in, thr ) );

  [sim,index] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

function [sim, out] = ld_vector_abs(sim, events, in, vecsize) // PARSEDOCU_BLOCK
// %PURPOSE: Vector abs()
//
// in *+(vecsize) - input
// out *+(vecsize) - output
// 
//    
  btype = 60001 + 52; 
  ipar = [vecsize; 0]; rpar = [];

  [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
                                     insizes=[vecsize], outsizes=[vecsize], ...
                                     intypes=[ ORTD.DATATYPE_FLOAT ], outtypes=[ORTD.DATATYPE_FLOAT] );
 
  // libdyn_conn_equation connects multiple input signals to blocks
  [sim,blk] = libdyn_conn_equation(sim, blk, list( in ) );

  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

function [sim, out] = ld_vectorInt32ToFloat(sim, events, in, vecsize) // PARSEDOCU_BLOCK
// %PURPOSE: Int32 to Float conversion on a vector 
//
// in *+(vecsize) - input
// out *+(vecsize) - output
// 
//    
  btype = 60001 + 83; 
  ipar = [vecsize; 0]; rpar = [];

  [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
                                     insizes=[vecsize], outsizes=[vecsize], ...
                                     intypes=[ ORTD.DATATYPE_INT32 ], outtypes=[ORTD.DATATYPE_FLOAT] );
 
  // libdyn_conn_equation connects multiple input signals to blocks
  [sim,blk] = libdyn_conn_equation(sim, blk, list( in ) );

  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

function [sim, out] = ld_vector_gain(sim, events, in, gain, vecsize) // PARSEDOCU_BLOCK   
// %PURPOSE: Vector gain
//
// in *+(vecsize) - input
// out *+(vecsize) - output
//    
  btype = 60001 + 53; 
  ipar = [vecsize]; rpar = [gain];

  [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
                                     insizes=[vecsize], outsizes=[vecsize], ...
                                     intypes=[ ORTD.DATATYPE_FLOAT ], outtypes=[ORTD.DATATYPE_FLOAT] );
 
  // libdyn_conn_equation connects multiple input signals to blocks
  [sim,blk] = libdyn_conn_equation(sim, blk, list( in ) );

  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

function [sim, out] = ld_vector_extract(sim, events, in, from, window_len, vecsize) // PARSEDOCU_BLOCK
//    
// %PURPOSE: Extract "in" from to from+window_len
// 
//  in *+(vecsize) - vector signal
//  from * - index signal, (indexing starts at 1)
//  out *+(window_len) - output signal
//
//    
  btype = 60001 + 54; 
  ipar = [vecsize; window_len]; rpar = [];

  [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
                                     insizes=[vecsize, 1], outsizes=[window_len], ...
                                     intypes=[ ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_FLOAT ], outtypes=[ORTD.DATATYPE_FLOAT] );
//   disp('new vextr\n');
//   disp( [vecsize, 1] );


  // libdyn_conn_equation connects multiple input signals to blocks
  [sim,blk] = libdyn_conn_equation(sim, blk, list( in, from ) );

  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

function [sim, index, value] = ld_vector_minmax(sim, events, in, findmax, vecsize) // PARSEDOCU_BLOCK
// %PURPOSE: Min / Max of a vector (finds the first appearance of the minimum/maximum)
//
// Function is buggy somehow. Find maximum should work. Minimum perhaps not!
//
// in *+(vecsize)
// findmax greater than 0 means "find the maximum"
// index * - the index starting at 1, where the max / min was found
// value * - min/max value
//    
  btype = 60001 + 55; 
  ipar = [vecsize; findmax]; rpar = [];

  [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
                                     insizes=[vecsize], outsizes=[1, 1], ...
                                     intypes=[ ORTD.DATATYPE_FLOAT ], outtypes=[ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_FLOAT] );
 
  // libdyn_conn_equation connects multiple input signals to blocks
  [sim,blk] = libdyn_conn_equation(sim, blk, list( in ) );

  [sim,index] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
  [sim,value] = libdyn_new_oport_hint(sim, blk, 1);   // 1th port
endfunction

function [sim, index, FoundSpike, Mean, Sigma, Distance, Val] = ld_vectorFindSpike(sim, events, in, SignificanceFactor, NskipLeft, NskipRight, vecsize) // PARSEDOCU_BLOCK
// %PURPOSE: find a spike in a given dataset
//
// Steps performed:
//
// 1) The maximum of abs(in) is determined as well as its position
// 2) The variance (sigma^2) of in is calculated except for the values surounded by
//    the maxmimum. This range is described by NskipLeft and NskipRight
// 3) The maximum is compared to the standard deviation (sigma); also the
//    signal's mean value is compensated herein.
// 4) If the intensity of the maximum is significantly higher than the maximum's
//    intensity, FoundSpike is set to 1 
//
// in *+(vecsize)
// SignificanceFactor - Used for the comparison Distance > SignificanceFactor * sigma, 
// index *(INT32) - the index starting at 1, where the spike was found
// FoundSpike *(INT32) - 1 if a spike has been found. 0 otherwise
//    
  btype = 60001 + 80; 
  ipar = [vecsize, NskipLeft, NskipRight]; rpar = [SignificanceFactor];

  [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
                                     insizes=[vecsize], outsizes=[1, 1, 1, 1, 1, 1], ...
                                     intypes=[ ORTD.DATATYPE_FLOAT ], ...
                                     outtypes=[ORTD.DATATYPE_INT32, ORTD.DATATYPE_INT32, ORTD.DATATYPE_FLOAT, ...
                                               ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_FLOAT ] );
 
  // libdyn_conn_equation connects multiple input signals to blocks
  [sim,blk] = libdyn_conn_equation(sim, blk, list( in ) );

  [sim,index] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
  [sim,FoundSpike] = libdyn_new_oport_hint(sim, blk, 1);   // 1th port
  [sim,Mean] = libdyn_new_oport_hint(sim, blk, 2);   // 1th port
  [sim,Sigma] = libdyn_new_oport_hint(sim, blk, 3);   // 1th port
  [sim,Distance] = libdyn_new_oport_hint(sim, blk, 4);   // 1th port
  [sim,Val] = libdyn_new_oport_hint(sim, blk, 5);   // 1th port
endfunction

function [sim, out, num] = ld_vector_glue(sim, events, in1, fromindex1, toindex1, in2, fromindex2, toindex2, vecsize) // PARSEDOCU_BLOCK
// %PURPOSE: Extract parts from two input vectors and glue them together to receive one vector.
//
// Output starting with in1 from fromindex1 until toindex1, continuing with in2 from fromindex2 until toindex2.
// Function is under developement!
//
// in1 *+(vecsize)
// in2 *+(vecsize)
// fromindex1 * - first index considered from in1
// toindex1 * - last index considered from in1
// fromindex2 * - first index considered from in2
// toindex2 * - last index considered from in2
// vecsize - size of each input vector. Vectors need to have equal size!
// out * - as explained above. Size of output is (2*vecsize).
// num * - number of values that have been glued together.
// 
//    
  btype = 60001 + 64;
  outsize = 2*vecsize;
  ipar = [vecsize]; rpar = [];

  [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
                                     insizes=[vecsize, 1, 1, vecsize, 1, 1], outsizes=[outsize, 1], ...
                                     intypes=[ORTD.DATATYPE_FLOAT,ORTD.DATATYPE_FLOAT,ORTD.DATATYPE_FLOAT,ORTD.DATATYPE_FLOAT,ORTD.DATATYPE_FLOAT,ORTD.DATATYPE_FLOAT], ...
                                     outtypes=[ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_FLOAT]);
 
  // libdyn_conn_equation connects multiple input signals to blocks
  [sim,blk] = libdyn_conn_equation(sim, blk, list( in1,fromindex1,toindex1,in2,fromindex2,toindex2 ) );

  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
  [sim,num] = libdyn_new_oport_hint(sim, blk, 1);   // 1th port
endfunction



function [sim, out] = ld_vector_addscalar(sim, events, in, add, vecsize) // PARSEDOCU_BLOCK
// %PURPOSE: add "add" to the vector
// 
//  add * - signal
//  in *+(vecsize) - vector signal
//    
  btype = 60001 + 56; 
  ipar = [vecsize]; rpar = [];

  [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
                                     insizes=[vecsize, 1], outsizes=[vecsize], ...
                                     intypes=[ ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_FLOAT  ], outtypes=[ORTD.DATATYPE_FLOAT] );
 
  // libdyn_conn_equation connects multiple input signals to blocks
  [sim,blk] = libdyn_conn_equation(sim, blk, list( in, add ) );

  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

function [sim, out] = ld_vector_add(sim, events, in1, in2, vecsize) // PARSEDOCU_BLOCK
// %PURPOSE: add two vectors elementwise
// 
//  in1 *+(vecsize) - vector signal1
//  in2 *+(vecsize) - vector signal2
//    
  btype = 60001 + 66; 
  ipar = [vecsize]; rpar = [];

  [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
                                     insizes=[vecsize, vecsize], outsizes=[vecsize], ...
                                     intypes=[ ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_FLOAT  ], outtypes=[ORTD.DATATYPE_FLOAT] );
 
  // libdyn_conn_equation connects multiple input signals to blocks
  [sim,blk] = libdyn_conn_equation(sim, blk, list( in1, in2 ) );

  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

function [sim, out] = ld_vector_sum(sim, events, in, vecsize) // PARSEDOCU_BLOCK
// %PURPOSE: sum over "in"
//
// in *+(vecsize)
// out *
//    
  btype = 60001 + 57;
  ipar = [vecsize]; rpar = [];

  [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
                                     insizes=[vecsize ], outsizes=[1], ...
                                     intypes=[ ORTD.DATATYPE_FLOAT ], outtypes=[ORTD.DATATYPE_FLOAT] );
 
  // libdyn_conn_equation connects multiple input signals to blocks
  [sim,blk] = libdyn_conn_equation(sim, blk, list( in ) );

  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

function [sim, out] = ld_vector_addsum(sim, events, in1, in2, vecsize) // FIXME TODO 
// %PURPOSE: multiplicate two vectors and calc the sum of the result ( sum( in1 .* in2) )
//
// in1 *+(vecsize)
// in2 *+(vecsize)
// out *
//    
  btype = 60001 + 58;
  ipar = [vecsize]; rpar = [];

  [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
                                     insizes=[vecsize,vecsize ], outsizes=[1], ...
                                     intypes=[ ORTD.DATATYPE_FLOAT,ORTD.DATATYPE_FLOAT ], outtypes=[ORTD.DATATYPE_FLOAT] );
 
  // libdyn_conn_equation connects multiple input signals to blocks
  [sim,blk] = libdyn_conn_equation(sim, blk, list( in1, in2 ) );

  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

function [sim, out] = ld_vector_abssum(sim, events, in, vecsize) // PARSEDOCU_BLOCK
// %PURPOSE: sum over element wise abs() of "in"
//
// in *+(vecsize)
// out *
//    
  btype = 60001 + 59;
  ipar = [vecsize]; rpar = [];

  [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
                                     insizes=[vecsize ], outsizes=[1], ...
                                     intypes=[ ORTD.DATATYPE_FLOAT ], outtypes=[ORTD.DATATYPE_FLOAT] );
 
  // libdyn_conn_equation connects multiple input signals to blocks
  [sim,blk] = libdyn_conn_equation(sim, blk, list( in ) );

  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

function [sim, out] = ld_vector_sqsum(sim, events, in, vecsize) // PARSEDOCU_BLOCK
// %PURPOSE: sum over element wise ()^2 of "in"
//
// in *+(vecsize)
// out *
//    
  btype = 60001 + 60;
  ipar = [vecsize]; rpar = [];

  [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
                                     insizes=[vecsize ], outsizes=[1], ...
                                     intypes=[ ORTD.DATATYPE_FLOAT ], outtypes=[ORTD.DATATYPE_FLOAT] );
 
  // libdyn_conn_equation connects multiple input signals to blocks
  [sim,blk] = libdyn_conn_equation(sim, blk, list( in ) );

  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

function [sim, out] = ld_vector_extractandsum(sim, events, in, from, window_len, vecsize) // PARSEDOCU_BLOCK
//    
// %PURPOSE: Extract "in" from "from"-index to "to"-index and sum up (untested) EXPERIMENTAL FOR NOW
// 
//  in *+(vecsize) - vector signal
//  from * - index signal
//  to * - index signal
//
//    
  btype = 60001 + 61;
  ipar = [vecsize]; rpar = [];

  [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
                                     insizes=[vecsize, 1, 1], outsizes=[1], ...
                                     intypes=[ ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_FLOAT ], outtypes=[ORTD.DATATYPE_FLOAT] );

  // libdyn_conn_equation connects multiple input signals to blocks
  [sim,blk] = libdyn_conn_equation(sim, blk, list( in, from, to ) );

  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

function [sim, out] = ld_simplecovar(sim, events, in, shape, vecsize) // PARSEDOCU_BLOCK
//    
// %PURPOSE: Cross correlation between a vectorial signal and a given shape, use ld_vecXCorrelation instead
// 
// Note: Use ld_vecXCorrelation instead. This will be removed soon.
//    
// The size of the output vector signal will be vecsize-length(shape) + 1

// FIXME: remove this function

  btype = 60001 + 62;
  ipar = [vecsize, length(shape) ]; rpar = [ shape ];

  if vecsize<length(shape) then
    error("vecsize<length(shape) !");
  end
  
  [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
                                     insizes=[vecsize  ], outsizes=[ vecsize-length(shape)+1 ], ...
                                     intypes=[ ORTD.DATATYPE_FLOAT ], outtypes=[ORTD.DATATYPE_FLOAT] );

  // libdyn_conn_equation connects multiple input signals to blocks
  [sim,blk] = libdyn_conn_equation(sim, blk, list( in ) );

  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

function [sim, out] = ld_vectorFindShape(sim, events, in, shape, vecsize) // PARSEDOCU_BLOCK
//    
// %PURPOSE: Cross correlation between a vectorial signal and a given shape like ld_vecXCorrelation using ofset compensation
// 
// Prior and after the floating window used to compare with the given shape
// the signal ofset of in at the current window position is estimated
//    
// The size of the output vector signal will be vecsize-length(shape) - 1

//pause;

  btype = 60001 + 79;
  ipar = [vecsize, length(shape) ]; rpar = [ shape ];

  if vecsize<( length(shape) + 2) then
    error("vecsize<length(shape) !");
  end
  
  OutVecSize = vecsize- (length(shape)+2) +1;
  
  [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
                                     insizes=[vecsize  ], outsizes=[  OutVecSize ], ...
                                     intypes=[ ORTD.DATATYPE_FLOAT ], outtypes=[ORTD.DATATYPE_FLOAT] );

  // printf("Output vector size: %f\n", OutVecSize );

  // libdyn_conn_equation connects multiple input signals to blocks
  [sim,blk] = libdyn_conn_equation(sim, blk, list( in ) );

  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

function [sim, out] = ld_vecXCorrelation(sim, events, in, shape, vecsize) // PARSEDOCU_BLOCK
//    
// %PURPOSE: Cross correlation between a vectorial signal and a given shape
// 
//  in *+(vecsize) - vector signal
//  shape - vector to compare the input with
//  out *+(vecsize-length(shape) + 1) - output
//
// Calculates the cross correlation between "in" and "shape"
//    
// The size of the output vector signal will be vecsize-length(shape) + 1

  btype = 60001 + 62;
  ipar = [vecsize, length(shape) ]; rpar = [ shape ];

  if vecsize<length(shape) then
    error("vecsize<length(shape) !");
  end
  
  [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
                                     insizes=[vecsize  ], outsizes=[ vecsize-length(shape)+1 ], ...
                                     intypes=[ ORTD.DATATYPE_FLOAT ], outtypes=[ORTD.DATATYPE_FLOAT] );

  // libdyn_conn_equation connects multiple input signals to blocks
  [sim,blk] = libdyn_conn_equation(sim, blk, list( in ) );

  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction


function [sim, out] = ld_vector_mute(sim, events, in, from, len, setto, vecsize) // PARSEDOCU_BLOCK
// %PURPOSE: mute a vector from and to a spacified index
// 
//  in *+(vecsize) - vector signal
//  from * - signal (index counting starts at )
//  len * - signal (length of the window to mute)
//  setto * - signal
//
//

  btype = 60001 + 63; 
  ipar = [vecsize]; rpar = [];

  [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
                                     insizes=[vecsize, 1, 1, 1, 1], outsizes=[vecsize], ...
                                     intypes=[ ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_FLOAT  ], outtypes=[ORTD.DATATYPE_FLOAT] );
 
  // libdyn_conn_equation connects multiple input signals to blocks
  [sim,blk] = libdyn_conn_equation(sim, blk, list( in, from, len, setto ) );

  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction



function [sim, out] = ld_vector_NaNtoVal(sim, events, in, Val, vecsize) // PARSEDOCU_BLOCK   
// %PURPOSE: Find all NaN in a vector and set them to Val
//
// in *+(vecsize) - input
// out *+(vecsize) - output
// Val - numeric parameter
//    
  btype = 60001 + 67; 
  ipar = [vecsize]; rpar = [Val];

  [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
                                     insizes=[vecsize], outsizes=[vecsize], ...
                                     intypes=[ ORTD.DATATYPE_FLOAT ], outtypes=[ORTD.DATATYPE_FLOAT] );
 
  // libdyn_conn_equation connects multiple input signals to blocks
  [sim,blk] = libdyn_conn_equation(sim, blk, list( in ) );
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

function [sim, outlist] = ld_LevelDemux(sim, events, in, NrEvents) // PARSEDOCU_BLOCK
//    
// %PURPOSE: Demux the level of the input signal such that the output corresponding to the input level is set to one
// 
//  in * - vector signal
//  outlist - list() of * with NrEvents elements
//
//  n = round(in) 
//  outlist(m) == 1, for n=m  AND outlist(m) == 0, for m != n
// 
//    

  KeepOutputLevel = 0;

  btype = 60001 + 68;
  ipar = [NrEvents, KeepOutputLevel]; rpar = [];

  [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
                                     insizes=[1], outsizes=[ ones(NrEvents,1) ], ...
                                     intypes=[ ORTD.DATATYPE_FLOAT ], outtypes=[ones(NrEvents,1)*ORTD.DATATYPE_FLOAT] );

  // libdyn_conn_equation connects multiple input signals to blocks
  [sim,blk] = libdyn_conn_equation(sim, blk, list( in ) );

  outlist = list();
  for i=1:NrEvents 
    [sim,out] = libdyn_new_oport_hint(sim, blk, i-1);   // 0th port
    outlist(i) = out;
  end
  
endfunction


function [sim, out] = ld_TrigSwitch1toN(sim, events, Event, SwitchInputList, InitialState) // PARSEDOCU_BLOCK
//    
// %PURPOSE: Switch N inputs to one output signal based on event pulses
// 
//  Event * - vector signal
//  SwitchInputList list() of * with N elements 
//
//  out * - is set to the input if SwitchInputList(state), whereby "state" is the current state that can be changed 
//          giving inpulses to Event, whose intensity correspond to the state to switch to. Event <0.5 does not change
//          the state.
// 
//    


  
  N = length(SwitchInputList);

  btype = 60001 + 69;
  ipar = [N, InitialState]; rpar = [];

  [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
                                     insizes=[1;ones(N,1)], outsizes=[ 1 ], ...
                                     intypes=[ ORTD.DATATYPE_FLOAT; ones(N,1)*ORTD.DATATYPE_FLOAT ], outtypes=[ORTD.DATATYPE_FLOAT] );

  // libdyn_conn_equation connects multiple input signals to blocks

  
  
  inlist = list(Event);
  
  for i=1:N
    inlist(i+1) = SwitchInputList(i);
  end
  [sim,blk] = libdyn_conn_equation(sim, blk, inlist );

  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
  
endfunction

function [sim, out] = ld_vector_concate(sim, events, in1, in2, size1, size2) // PARSEDOCU_BLOCK
  //
  // %PURPOSE: Concatenate two vectors
  //
  // in1, in2 *+ - input vectors
  // out *(size1+size2) - the concatenated vector
  // size1, size2 - respective length for both vectors
  // 
  //

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'in1', in1) );
  ortd_checkpar(sim, list('Signal', 'in2', in2) );
  ortd_checkpar(sim, list('SingleValue', 'size1', size1) );
  ortd_checkpar(sim, list('SingleValue', 'size2', size2) );
end

  btype = 60001 + 70; 
  ipar = [ size1, size2 ]; rpar = [];

  [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
                       insizes=[size1, size2], outsizes=[size1+size2], ...
                       intypes=[ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_FLOAT], outtypes=[ORTD.DATATYPE_FLOAT]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list(in1, in2) );

  // connect each outport
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // ith port
endfunction

function [sim, out] = ld_vector_multscalar(sim, events, in, mult, vecsize) // PARSEDOCU_BLOCK
// %PURPOSE: multiplicate the given vector
// 
//  mult * - signal
//  in *+(vecsize) - vector signal
//    

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'in', in) );
  ortd_checkpar(sim, list('Signal', 'mult', mult) );
  ortd_checkpar(sim, list('SingleValue', 'vecsize', vecsize) );
end

  btype = 60001 + 71; 
  ipar = [vecsize]; rpar = [];

  [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
                                     insizes=[vecsize, 1], outsizes=[vecsize], ...
                                     intypes=[ ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_FLOAT  ], outtypes=[ORTD.DATATYPE_FLOAT] );
 
  // libdyn_conn_equation connects multiple input signals to blocks
  [sim,blk] = libdyn_conn_equation(sim, blk, list( in, mult ) );

  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction



function [sim, out] = ld_collectValues(sim, events, in, WriteIndex, memorysize, DefaultVal, inVecsize ) // PARSEDOCU_BLOCK
  //
  // %PURPOSE: Store input values in a memory at a given position
  //
  // in * - input vector whose values shall be stored
  // out *(memorysize) - the vector representing the memory
  // WriteIndex * INT32 - the index to write the data to; starts at 1
  // memorysize - storage size
  // DefaultVal - initialize the storage with this value
  // vecsize - size of the input vector
  //

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'in', in) );
  ortd_checkpar(sim, list('Signal', 'WriteIndex', WriteIndex) );
  ortd_checkpar(sim, list('SingleValue', 'memorysize', memorysize) );
  ortd_checkpar(sim, list('SingleValue', 'DefaultVal', DefaultVal) );
  ortd_checkpar(sim, list('SingleValue', 'inVecsize', inVecsize) );
end

  btype = 60001 + 74;
  ipar = [ memorysize, inVecsize ]; rpar = [DefaultVal];

  [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
                       insizes=[inVecsize, 1 ], outsizes=[memorysize], ...
                       intypes=[ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_INT32], outtypes=[ORTD.DATATYPE_FLOAT]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list(in, WriteIndex) );

  // connect each outport
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // ith port
endfunction


function [sim, out] = ld_HistogramInt32(sim, events, Val, Weight, from, to ) // PARSEDOCU_BLOCK
  //
  // %PURPOSE: Accumulative store input values in a memory at a given position
  //
  // Val * - input 
  // Weight * - int32
  // out *(from-to+1) int32 - the vector representing the histogram
  // 
  // from - min input value in the histogram
  // to - max input value in the histogram
  // 
  //

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'Val', Val) );
  ortd_checkpar(sim, list('Signal', 'Weight', Weight) );
  ortd_checkpar(sim, list('SingleValue', 'from', from) );
  ortd_checkpar(sim, list('SingleValue', 'to', to) );
end

  btype = 60001 + 81;
  ipar = [ from, to ]; rpar = [];
  
//  pause;

  [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
                       insizes=[1, 1 ], outsizes=[to-from+1], ...
                       intypes=[ORTD.DATATYPE_INT32, ORTD.DATATYPE_INT32], outtypes=[ORTD.DATATYPE_INT32]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list(Val, Weight) );

  // connect each outport
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // ith port
endfunction


// [sim, TimerActive, Counter] = ld_Timer(sim, 0, Trigger=AbnormalityDetected, Count=length(par_.CorrModel) )

function [sim, TimerActive, Counter] = ld_Timer(sim, events, Trigger, Count ) // PARSEDOCU_BLOCK
  //
  // %PURPOSE: A timer than can be triggered
  //
  // The timer is active for Count simulation steps
  //
  // Trigger * - (int32) start the timer immediately 
  // TimerActive * - int32 1 if the timer is active, 0 if not
  // Counter * (int32) - The used conter that starts from Count and decreases
  // 
  // 
  //

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'Trigger', Trigger) );
  ortd_checkpar(sim, list('SingleValue', 'Count', Count) );
end

  btype = 60001 + 82;
  ipar = [ Count ]; rpar = [];
  
//  pause;

  [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
                       insizes=[1 ], outsizes=[1,1], ...
                       intypes=[ORTD.DATATYPE_INT32], outtypes=[ORTD.DATATYPE_INT32, ORTD.DATATYPE_INT32]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list(Trigger) );

  // connect each outport
  [sim, TimerActive] = libdyn_new_oport_hint(sim, blk, 0);   // ith port
  [sim, Counter] = libdyn_new_oport_hint(sim, blk, 1);   // ith port
endfunction



function [sim,out] = ld_add_ofsInt32(sim, events, in, ofs) // PARSEDOCU_BLOCK
//
// %PURPOSE: add ofs(in)
// 
// input and return value are of type ORTD.DATATYPE_INT32
// 

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'in', in) );
  ortd_checkpar(sim, list('SingleValue', 'ofs', ofs) );
end

  btype = 60001 + 75;
  [sim,blk] = libdyn_new_block(sim, events, btype, [ ofs ], [  ], ...
                   insizes=[1], outsizes=[1], ...
                   intypes=[ORTD.DATATYPE_INT32], outtypes=[ORTD.DATATYPE_INT32]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list(in) );
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

function [sim,out] = ld_gainInt32(sim, events, in, fac) // PARSEDOCU_BLOCK
//
// %PURPOSE: integer multiplication of (in)
// 
// input and return value are of type ORTD.DATATYPE_INT32
// 

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'in', in) );
  ortd_checkpar(sim, list('SingleValue', 'fac', fac) );
end

  btype = 60001 + 76;
  [sim,blk] = libdyn_new_block(sim, events, btype, [ fac ], [  ], ...
                   insizes=[1], outsizes=[1], ...
                   intypes=[ORTD.DATATYPE_INT32], outtypes=[ORTD.DATATYPE_INT32]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list(in) );
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction



function [sim,out] = ld_VarVec_ztf(sim, events, in, Nvalues, H, FilterMode, vecsize) // PARSEDOCU_BLOCK
//
// %PURPOSE: discrete-time transfer function applied to vector data
// 
// FilterMode == 1 apply filter from left to right
// FilterMode == 2 apply filter from right to left
// FilterMode == 3 apply filter from left to right, flip result, filter from right to left (zero phase-shift filter)
// 
// H is a transfer function in z-domain represented by a scilab rational
//
// in (float, vecsize) - vector input
// out (float, vecsize) - vector output
// Nvalues (int32) - the number of samples the filter is applied to (starting from the left side of the vector)
//

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'in', in) );
  ortd_checkpar(sim, list('Signal', 'Nvalues', Nvalues) );
  
  ortd_checkpar(sim, list('SingleValue', 'vecsize', vecsize) );
  ortd_checkpar(sim, list('SingleValue', 'FilterMode', FilterMode) );
end

//  bip = [ degree(H.num); degree(H.den) ];
//  brp = [ coeff(H.num)'; coeff(H.den)' ];

  btype = 60001 + 85;
  [sim,blk] = libdyn_new_block(sim, events, btype, [ degree(H.num), degree(H.den) , vecsize, FilterMode ], [ coeff(H.num)'; coeff(H.den)' ], ...
                   insizes=[ vecsize, 1 ], outsizes=[ vecsize ], ...
                   intypes=[ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_INT32], outtypes=[ORTD.DATATYPE_FLOAT]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list(in, Nvalues) );
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction


function [sim,out] = ld_VarVec_add(sim, events, inlist, Nvalues, weight, vecsize) // PARSEDOCU_BLOCK
//
// %PURPOSE: Linear combination of two vectors
// 
// 
//
// inlist (float, vecsize) - list() of vector input
// out (float, vecsize) - vector output
// Nvalues (int32) - the number of samples the filter is applied to (starting from the left side of the vector)
//

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'inlist(1)', inlist(1) ) );
  ortd_checkpar(sim, list('Signal', 'inlist(2)', inlist(2) ) );
  ortd_checkpar(sim, list('Signal', 'Nvalues', Nvalues) );
  
  ortd_checkpar(sim, list('SingleValue', 'vecsize', vecsize) );
  ortd_checkpar(sim, list('SingleValue', 'weight(1)', weight(1) ) );
  ortd_checkpar(sim, list('SingleValue', 'weight(2)', weight(2) ) );
end

//  bip = [ degree(H.num); degree(H.den) ];
//  brp = [ coeff(H.num)'; coeff(H.den)' ];

  btype = 60001 + 87;
  [sim,blk] = libdyn_new_block(sim, events, btype, [  vecsize ], [ weight(1), weight(2) ], ...
                   insizes=[ vecsize, vecsize, 1 ], outsizes=[ vecsize ], ...
                   intypes=[ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_INT32], outtypes=[ORTD.DATATYPE_FLOAT]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list(inlist(1), inlist(2), Nvalues) );
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction


function [sim, Mean, Sigma] = ld_VarVec_Variance(sim, events, in, Nvalues, vecsize) // PARSEDOCU_BLOCK
//
// %PURPOSE: Calc variance of vector elements
// 
// 
//
// in (float, vecsize) - vector input
// out (float, vecsize) - vector output
// Nvalues (int32) - the number of samples the filter is applied to (starting from the left side of the vector)
//

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'in', in ) );
  ortd_checkpar(sim, list('Signal', 'Nvalues', Nvalues) );
  
  ortd_checkpar(sim, list('SingleValue', 'vecsize', vecsize) );
end



  btype = 60001 + 88;
  [sim,blk] = libdyn_new_block(sim, events, btype, [  vecsize ], [ ], ...
                   insizes=[ vecsize, 1 ], outsizes=[ 1,1 ], ...
                   intypes=[ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_INT32], outtypes=[ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_FLOAT]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list(in, Nvalues) );
  [sim,Mean] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
  [sim,Sigma] = libdyn_new_oport_hint(sim, blk, 1);   // 0th port
endfunction


function [sim, out] = ld_VarVec_AbsSumNorm(sim, events, in, Nvalues, vecsize) // PARSEDOCU_BLOCK
//
// %PURPOSE: Calc sum of the absolute value of vector elements
// 
// 
//
// in (float, vecsize) - vector input
// out (float, vecsize) - vector output
// Nvalues (int32) - the number of samples the filter is applied to (starting from the left side of the vector)
//

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'in', in ) );
  ortd_checkpar(sim, list('Signal', 'Nvalues', Nvalues) );
  
  ortd_checkpar(sim, list('SingleValue', 'vecsize', vecsize) );
end



  btype = 60001 + 90;
  [sim,blk] = libdyn_new_block(sim, events, btype, [  vecsize ], [ ], ...
                   insizes=[ vecsize, 1 ], outsizes=[ 1 ], ...
                   intypes=[ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_INT32], outtypes=[ORTD.DATATYPE_FLOAT]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list(in, Nvalues) );
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction



function [sim, Min, Max] = ld_VarVec_MinMax(sim, events, in, Nvalues, vecsize) // PARSEDOCU_BLOCK
//
// %PURPOSE: Calc Min and Max of vector elements
// 
// 
//
// in (float, vecsize) - vector input
// out (float, vecsize) - vector output
// Nvalues (int32) - the number of samples the filter is applied to (starting from the left side of the vector)
//

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'in', in ) );
  ortd_checkpar(sim, list('Signal', 'Nvalues', Nvalues) );
  
  ortd_checkpar(sim, list('SingleValue', 'vecsize', vecsize) );
end



  btype = 60001 + 91;
  [sim,blk] = libdyn_new_block(sim, events, btype, [  vecsize ], [ ], ...
                   insizes=[ vecsize, 1 ], outsizes=[ 1,1 ], ...
                   intypes=[ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_INT32], outtypes=[ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_FLOAT]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list(in, Nvalues) );
  [sim,Min] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
  [sim,Max] = libdyn_new_oport_hint(sim, blk, 1);   // 0th port
endfunction

//function [sim, out] = ld_VarVec_Abs(sim, events, in, Nvalues, vecsize) // PARSEDOCU_BLOCK
////
//// %PURPOSE: Calc the absolute values of vector elements
//// 
//// 
////
//// in (float, vecsize) - vector input
//// out (float, vecsize) - vector output
//// Nvalues (int32) - the number of samples the filter is applied to (starting from the left side of the vector)
////
//
//if ORTD.FASTCOMPILE==%f then
//  ortd_checkpar(sim, list('Signal', 'in', in ) );
//  ortd_checkpar(sim, list('Signal', 'Nvalues', Nvalues) );
//  
//  ortd_checkpar(sim, list('SingleValue', 'vecsize', vecsize) );
//end
//
//
//
//  btype = 60001 + 92;
//  [sim,blk] = libdyn_new_block(sim, events, btype, [  vecsize ], [ ], ...
//                   insizes=[ vecsize, 1 ], outsizes=[ vecsize ], ...
//                   intypes=[ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_INT32], outtypes=[ORTD.DATATYPE_FLOAT]  );
//
//  [sim,blk] = libdyn_conn_equation(sim, blk, list(in, Nvalues) );
//  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
//endfunction
//



function [sim, out, Nvalues] = ld_vector_VarExtract(sim, events, in, from, to, vecsize) // PARSEDOCU_BLOCK
//    
// %PURPOSE: Extract vector elements from a window variable in size 
// 
//  in *+(vecsize) - vector signal
//  from, to (INT32) - cut parameters, indices start at 1
//  out *+(window_len) - output signal
//  Nvalues (INT32) - number of the elements cut
//    
  btype = 60001 + 86; 
  ipar = [vecsize]; rpar = [];

  [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
                                     insizes=[vecsize, 1, 1], outsizes=[vecsize, 1], ...
                                     intypes=[ ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_INT32, ORTD.DATATYPE_INT32 ], ....
                                     outtypes=[ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_INT32] );

  // libdyn_conn_equation connects multiple input signals to blocks
  [sim,blk] = libdyn_conn_equation(sim, blk, list( in, from, to ) );

  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
  [sim,Nvalues] = libdyn_new_oport_hint(sim, blk, 1);   // 1th port
endfunction






function [sim, out] = ld_switch2to1Int32(sim, events, cntrl, in1, in2) // PARSEDOCU_BLOCK
//
// %PURPOSE: A 2 to 1 switching Block
//
// cntr (INT32) - control input
// in1 (INT32)
// in2 (INT32)
// out (INT32) - output
//
// if cntrl > (greather than) : out = in1
//   else                     : out = in2
//

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'cntrl', cntrl) );
  ortd_checkpar(sim, list('Signal', 'in1', in1) );
  ortd_checkpar(sim, list('Signal', 'in2', in2) );
end


  btype = 60001 + 89;
  [sim,blk] = libdyn_new_block(sim, events, btype, [], [], ...
                   insizes=[1, 1, 1], outsizes=[1], ...
                   intypes=[ORTD.DATATYPE_INT32, ORTD.DATATYPE_INT32, ORTD.DATATYPE_INT32], ...
                   outtypes=[ORTD.DATATYPE_INT32]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list(cntrl, in1, in2) );
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

function [sim, out] = ld_max(sim, events, inlist) // PARSEDOCU_BLOCK
// %PURPOSE: logic and - block
//
// in *LIST - list() of inputs (for now the exactly two inputs are possible)
// out * - output
// 
// 

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('SignalList', 'inlist', inlist) );
end

  Nin=length(inlist);

  if (Nin ~= 2) then
    error("invalid number of inputs");
  end

  insizes=ones(1, Nin);
  intypes=ones(1, Nin) * ORTD.DATATYPE_FLOAT;

  btype = 60001 + 92;
  [sim,blk] = libdyn_new_block(sim, events, btype, ipar=[  ], rpar=[   ], ...
                   insizes, outsizes=[1], ...
                   intypes, outtypes=[ORTD.DATATYPE_FLOAT]  );

  [sim,blk] = libdyn_conn_equation(sim, blk, list( inlist(1), inlist(2) ) );
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction


//
//// FIXME no port size checking
//function [sim,bid] = libdyn_new_blk_zTF(sim, events, H)
//  btype = 30;
//  bip = [ degree(H.num); degree(H.den) ];
//  brp = [ coeff(H.num)'; coeff(H.den)' ];
//
//  [sim,bid] = libdyn_new_block(sim, events, btype, [bip], [brp], ...
//                   insizes=[1], outsizes=[1], ...
//                   intypes=[ORTD.DATATYPE_FLOAT], outtypes=[ORTD.DATATYPE_FLOAT]);
//  
////  [sim,bid] = libdyn_new_blockid(sim);
////  id = bid; // id for this parameter set
////  
////  
////  sim.parlist = new_irparam_elemet(sim.parlist, id, IRPAR_LIBDYN_BLOCK, [btype; bid; bip], [brp]);
//endfunction
//function [sim,y] = ld_ztf(sim, events, inp_list, H) // PARSEDOCU_BLOCK
////
//// %PURPOSE: Time discrete transfer function
//// H is give as a Scilab rational
////
//
//if ORTD.FASTCOMPILE==%f then
//  ortd_checkpar(sim, list('Signal', 'inp_list', inp_list) );
//end
//
//  [inp] = libdyn_extrakt_obj( inp_list ); // compatibility
//
//    [sim,tf] = libdyn_new_blk_zTF(sim, events, H);
//    
//    
//    
//    [sim,y] = libdyn_conn_equation(sim, tf, list(inp));
//    [sim,y] = libdyn_new_oport_hint(sim, y, 0);    
//endfunction


// 
// 
// 
// 
// Blocks that use the new Cpp Interfac
// 
// 
// 
// 



function [sim, out] = ld_RTCrossCorr(sim, events, u, shapeSig, len) // PARSEDOCU_BLOCK
// 
// Online Cross Correlation
//
// u * - input signal
// shapeSig * - input shape
// out * - output signal
// len - length of input shape
// 
// 
// Note: The implementation is not optimal. Only the raw sum formula is evaluated.
// 

// introduce some parameters that are refered to by id's

// Set-up the block parameters and I/O ports
  Uipar = [ ];
  Urpar = [ ];
  btype = 60001 + 300;

  insizes=[len,1]; // Input port sizes
  outsizes=[1]; // Output port sizes
  dfeed=[1];  // for each output 0 (no df) or 1 (a direct feedthrough to one of the inputs)
  intypes=[ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_FLOAT]; // datatype for each input port
  outtypes=[ORTD.DATATYPE_FLOAT]; // datatype for each output port

  blocktype = 1; // 1-BLOCKTYPE_DYNAMIC (if block uses states), 2-BLOCKTYPE_STATIC (if there is only a static relationship between in- and output)

  // Create the block
  [sim, blk] = libdyn_CreateBlockAutoConfig(sim, events, btype, blocktype, Uipar, Urpar, insizes, outsizes, intypes, outtypes, dfeed);
  
  // connect the inputs
 [sim,blk] = libdyn_conn_equation(sim, blk, list(shapeSig, u) ); // connect in1 to port 0 and in2 to port 1

  // connect the ouputs
 [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction




function [sim, out] = ld_ReadAsciiFile(sim, events, fname, veclen) // PARSEDOCU_BLOCK
// 
// Read data from an ascii-file
//
// fname - file name (string)
// veclen - Size of the vector to read during each time-step
// out *(veclen) - Output signal as read from the file
// 
// The data contained in the file must be ascii data divided into rows and columns
// as it may be read on Scilab using the command "fscanfMat".
// The number of columns must be veclen. 
// For each time-step the respectively next row of data is read from the file and
// copied to the block's output vectorial signal "out"
// 
// The file must be available only at runtime.
// 
// 


   // pack all parameters into a structure "parlist"
   parlist = new_irparam_set();

   parlist = new_irparam_elemet_ivec(parlist, ascii(fname), 12); // id = 12; A string parameter

   p = combine_irparam(parlist); // convert to two vectors of integers and floating point values respectively

// Set-up the block parameters and I/O ports
  Uipar = [ p.ipar ];
  Urpar = [ p.rpar ];
  btype = 60001 + 301; // Reference to the block's type (computational function). Use the same id you are giving via the "libdyn_compfnlist_add" C-function

  insizes=[]; // Input port sizes
  outsizes=[veclen]; // Output port sizes
  dfeed=[1];  // for each output 0 (no df) or 1 (a direct feedthrough to one of the inputs)
  intypes=[]; // datatype for each input port
  outtypes=[ORTD.DATATYPE_FLOAT]; // datatype for each output port

  blocktype = 1; // 1-BLOCKTYPE_DYNAMIC (if block uses states), 2-BLOCKTYPE_STATIC (if there is only a static relationship between in- and output)

  // Create the block
  [sim, blk] = libdyn_CreateBlockAutoConfig(sim, events, btype, blocktype, Uipar, Urpar, insizes, outsizes, intypes, outtypes, dfeed);
  
  // connect the inputs
//  [sim,blk] = libdyn_conn_equation(sim, blk, list(in1, in2) ); // connect in1 to port 0 and in2 to port 1

  // connect the ouputs
 [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

function [sim, out] = ld_ArrayInt32(sim, events, array, in) // PARSEDOCU_BLOCK
// 
// Lookup a value inside an array - block
//
// in * - input
// out * - output
// 
// out = array[in]
// 

if ORTD.FASTCOMPILE==%f then
   ortd_checkpar(sim, list('Signal', 'in', in) );
  //FIXME check array
end

// introduce some parameters that are refered to by id's

   // pack all parameters into a structure "parlist"
   parlist = new_irparam_set();

   parlist = new_irparam_elemet_ivec(parlist, array, 10); // id = 10

   p = combine_irparam(parlist); // convert to two vectors of integers and floating point values respectively

// Set-up the block parameters and I/O ports
  Uipar = [ p.ipar ];
  Urpar = [ p.rpar ];
  btype = 60001 + 303; // Reference to the block's type (computational function). Use the same id you are giving via the "libdyn_compfnlist_add" C-function

  insizes=[1]; // Input port sizes
  outsizes=[1]; // Output port sizes
  dfeed=[1];  // for each output 0 (no df) or 1 (a direct feedthrough to one of the inputs)
  intypes=[ORTD.DATATYPE_INT32]; // datatype for each input port
  outtypes=[ORTD.DATATYPE_INT32]; // datatype for each output port

  blocktype = 2; // 1-BLOCKTYPE_DYNAMIC (if block uses states), 2-BLOCKTYPE_STATIC (if there is only a static relationship between in- and output)

  // Create the block
  [sim, blk] = libdyn_CreateBlockAutoConfig(sim, events, btype, blocktype, Uipar, Urpar, insizes, outsizes, intypes, outtypes, dfeed);
  
  // connect the inputs
 [sim,blk] = libdyn_conn_equation(sim, blk, list(in) ); // connect in1 to port 0 and in2 to port 1

  // connect the ouputs
 [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction



function [sim, out] = ld_const_bin(sim, events, BinConst) // PARSEDOCU_BLOCK
// 
// %PURPOSE: a constant vector of ORTD.DATATYPE_BINARY
// 
// out *+ - the vector of binary
// 


  
  //ortd_checkpar(sim, list('', 'BinConst', BinConst) );

  // pack all parameters into a structure "parlist"
  parlist = new_irparam_set();
  insize = length(BinConst);
  parlist = new_irparam_elemet_ivec(parlist, insize, 10); // id = 10
  parlist = new_irparam_elemet_ivec(parlist, BinConst, 11); // id = 11
  
  p = combine_irparam(parlist); // convert to two vectors of integers and floating point values respectively

  // Set-up the block parameters and I/O ports
  Uipar = [ p.ipar ];
  Urpar = [ p.rpar ];
  btype = 60001 + 304; // Reference to the block's type (computational function). Use the same id you are giving via the "libdyn_compfnlist_add" C-function

  insizes=[];
  outsizes=[ insize ]; // Output port sizes
  dfeed=[1];  // for each output 0 (no df) or 1 (a direct feedthrough to one of the inputs)
  intypes=[];
  outtypes=[ ORTD.DATATYPE_BINARY  ]; // datatype for each output port

  blocktype = 2; // 1-BLOCKTYPE_DYNAMIC (if block uses states), 2-BLOCKTYPE_STATIC (if there is only a static relationship between in- and output)

  // Create the block
  [sim, blk] = libdyn_CreateBlockAutoConfig(sim, events, btype, blocktype, Uipar, Urpar, insizes, outsizes, intypes, outtypes, dfeed);
  

  // connect the ouputs
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

function [sim] = ld_ORTDIO_Put(sim, events, in, len, datatype, header) // PARSEDOCU_BLOCK
// 
// Put data to ORTD_IO
//
// len - Size of the vector to be send
// in *(len) - Input signal.
// datatype - Datatype of signal "in"
// header - A string that is prepended to each binary message
// 


   // pack all parameters into a structure "parlist"
   parlist = new_irparam_set();

   parlist = new_irparam_elemet_ivec(parlist, ascii(header), 12); // id = 12; A string parameter

   p = combine_irparam(parlist); // convert to two vectors of integers and floating point values respectively

// Set-up the block parameters and I/O ports
  Uipar = [ p.ipar ];
  Urpar = [ p.rpar ];
  btype = 60001 + 305; // Reference to the block's type (computational function). Use the same id you are giving via the "libdyn_compfnlist_add" C-function

  insizes=[len]; // Input port sizes
  outsizes=[]; // Output port sizes
  dfeed=[1];  // for each output 0 (no df) or 1 (a direct feedthrough to one of the inputs)
  intypes=[datatype]; // datatype for each input port
  outtypes=[]; // datatype for each output port

  blocktype = 1; // 1-BLOCKTYPE_DYNAMIC (if block uses states), 2-BLOCKTYPE_STATIC (if there is only a static relationship between in- and output)

  // Create the block
  [sim, blk] = libdyn_CreateBlockAutoConfig(sim, events, btype, blocktype, Uipar, Urpar, insizes, outsizes, intypes, outtypes, dfeed);
  
  // connect the inputs
  [sim,blk] = libdyn_conn_equation(sim, blk, list(in) ); // 

  // connect the ouputs
 //[sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

function [sim] = ld_SyncFilewrite(sim, events, in, len, datatype, fname, trigger, par) // PARSEDOCU_BLOCK
// 
// Synchronously write ascii data to a file
//
// Data is stored directly during the flag for updating states and may hence disturbe realtime operation
// of a surrounding realtime loop. Only datatype float is currently supported.
//
// len - Size of the vector to be send
// in *(len) - Input signal.
// datatype - Datatype of signal "in"
// fname - A string that is prepended to each binary message
// par - optional parameters (none at the moment. Put par=struct() )
// 


   // pack all parameters into a structure "parlist"
   parlist = new_irparam_set();

   parlist = new_irparam_elemet_ivec(parlist, ascii(fname), 12); // id = 12; A string parameter

   p = combine_irparam(parlist); // convert to two vectors of integers and floating point values respectively

// Set-up the block parameters and I/O ports
  Uipar = [ p.ipar ];
  Urpar = [ p.rpar ];
  btype = 60001 + 306; // Reference to the block's type (computational function). Use the same id you are giving via the "libdyn_compfnlist_add" C-function

  insizes=[len,1]; // Input port sizes
  outsizes=[]; // Output port sizes
  dfeed=[1];  // for each output 0 (no df) or 1 (a direct feedthrough to one of the inputs)
  intypes=[datatype, ORTD.DATATYPE_INT32]; // datatype for each input port
  outtypes=[]; // datatype for each output port

  blocktype = 1; // 1-BLOCKTYPE_DYNAMIC (if block uses states), 2-BLOCKTYPE_STATIC (if there is only a static relationship between in- and output)

  // Create the block
  [sim, blk] = libdyn_CreateBlockAutoConfig(sim, events, btype, blocktype, Uipar, Urpar, insizes, outsizes, intypes, outtypes, dfeed);
  
  // connect the inputs
  [sim,blk] = libdyn_conn_equation(sim, blk, list(in, trigger) ); // 

  // connect the ouputs
 //[sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction


function [sim, EstMean, EstSigma] = ld_WindowedMean(sim, events, in, weight, WindowLen) // PARSEDOCU_BLOCK
// 
// ld_WindowedMean - Calculte the average of the input singal for a floating window
//
// in * - input signal
// weight * - weight for the current sample
// EstMean * - the calculated average
// EstSigma * - the calculated standard deviation
// WindowLen - length of floating window
// 
// If weight is less than 0 the filter update is prevented. E.g. if in contains an invalid sample that
// shall not be counted in the calculation of the Mean and Variance
// 


// introduce some parameters that are refered to by id's

// Set-up the block parameters and I/O ports
  Uipar = [ WindowLen ];
  Urpar = [ ];
  btype = 60001 + 307;

  insizes=[1,1]; // Input port sizes
  outsizes=[1,1]; // Output port sizes
  dfeed=[1];  // for each output 0 (no df) or 1 (a direct feedthrough to one of the inputs)
  intypes=[ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_FLOAT]; // datatype for each input port
  outtypes=[ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_FLOAT]; // datatype for each output port

  blocktype = 1; // 1-BLOCKTYPE_DYNAMIC (if block uses states), 2-BLOCKTYPE_STATIC (if there is only a static relationship between in- and output)

  // Create the block
  [sim, blk] = libdyn_CreateBlockAutoConfig(sim, events, btype, blocktype, Uipar, Urpar, insizes, outsizes, intypes, outtypes, dfeed);
  
  // connect the inputs
 [sim,blk] = libdyn_conn_equation(sim, blk, list(in, weight) ); // connect in1 to port 0 and in2 to port 1

  // connect the ouputs
 [sim,EstMean] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
 [sim,EstSigma] = libdyn_new_oport_hint(sim, blk, 1);   // 0th port
 
endfunction








// 
// 
// Special blocks
// 
// 


// obsolete
function [sim,bid] = libdyn_new_interface(sim, events, len)
  btype = 4000;
  [sim,bid] = libdyn_new_blk_generic(sim, events, btype, [len], []);
endfunction


function [sim, out] = ld_interface(sim, events, in, vecsize) // PARSEDOCU_BLOCK   
// Interfacing block
//
// in *+(vecsize) - input
// out *+(vecsize) - output
//    
  btype = 60001 + 1000; 
  ipar = [vecsize]; rpar = [];

  [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
                                     insizes=[vecsize], outsizes=[vecsize], ...
                                     intypes=[ ORTD.DATATYPE_FLOAT ], outtypes=[ORTD.DATATYPE_FLOAT] );
 
  // libdyn_conn_equation connects multiple input signals to blocks
  [sim,blk] = libdyn_conn_equation(sim, blk, list( in ) );

  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction





//
// Macros
//
//


function [sim,y] = ld_add_ofs(sim, events, u, ofs) // PARSEDOCU_BLOCK
//
// %PURPOSE: Add a constant "ofs" to the signal u; y = u + const(ofs)
//

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'u', u) );
  ortd_checkpar(sim, list('SingleValue', 'ofs', ofs) );
end

  [sim,ofs_const] = libdyn_new_blk_const(sim, events, ofs);
  
  [sim,y] = ld_sum(sim, events, list(u,0, ofs_const,0), 1,1);
endfunction


function [sim, y] = ld_mute( sim, ev, u, cntrl, mutewhengreaterzero ) // PARSEDOCU_BLOCK
//    
//    %PURPOSE: Mute a signal based on cntrl-signal
//
//    ev - event
//    u * - input
//    y * - output
//    mutewhengreaterzero - boolean parameter (%T, %F)
//    
//    if mutewhengreaterzero == %T then
//
//      y = 0 for cntrl > 0
//      y = u for cntrl < 0
//
//    else
//
//      y = 0 for cntrl < 0
//      y = u for cntrl > 0
//
//
//
    
    [sim, zero] = ld_const(sim, ev, 0);
    
    if (mutewhengreaterzero == %T) then
      [sim,y] = ld_switch2to1(sim, ev, cntrl, zero, u);
    else
      [sim,y] = ld_switch2to1(sim, ev, cntrl, u, zero);
    end
endfunction



function [sim, y] = ld_limited_integrator(sim, ev, u, min__, max__, Ta) // PARSEDOCU_BLOCK
// %PURPOSE: Implements a time discrete integrator with saturation of the output between min__ and max__
// 
// u * - input
// y * - output
// 
// y(k+1) = sat( y(k) + Ta*u , min__, max__ )
    [sim, u__] = ld_gain(sim, ev, u, Ta);
    
    [sim,z_fb] = libdyn_new_feedback(sim);
    
  [sim, sum_] = ld_sum(sim, ev, list(u__, z_fb), 1, 1);
  [sim, tmp] = ld_ztf(sim, ev, sum_, 1/z);
  [sim, y] = ld_sat(sim, ev, tmp, min__, max__);
    
    [sim] = libdyn_close_loop(sim, y, z_fb);    
endfunction

function [sim, y] = ld_limited_integrator2(sim, ev, u, min__, max__, Ta) // PARSEDOCU_BLOCK
// %PURPOSE: Implements a time discrete integrator with saturation of the output between min__ and max__
// compared to ld_limited_integrator there is no delay: Ta z / (z-1)
//
// u * - input
// y * - output
// 
// y(k+1) = sat( y(k) + Ta*u , min__, max__ )


    [sim, u__] = ld_gain(sim, ev, u, Ta);
    
    [sim,z_fb] = libdyn_new_feedback(sim);
    
  [sim, sum_] = ld_sum(sim, ev, list(u__, z_fb), 1, 1);
  [sim, y] = ld_sat(sim, ev, sum_, min__, max__);


  [sim, y_] = ld_ztf(sim, ev, y, 1/z);
    
    [sim] = libdyn_close_loop(sim, y_, z_fb);    

endfunction

function [sim, y] = ld_limited_integrator3(sim, ev, u, min__, max__, Ta) // PARSEDOCU_BLOCK
// %PURPOSE: Implements a time discrete integrator (trapeziodal rule) with saturation of the output between min__ and max__
//
// u * - input
// y * - output
// 
// y(k+1) = sat(  Ta/2 ( u(k)+u(k+1) ) + y(k), min__, max__ )

    ukp1 = u;
    [sim, uk] = ld_ztf(sim, ev, ukp1, 1/z);
    [sim, u__] = ld_add(sim, ev, list(uk, ukp1), [Ta/2, Ta/2] );     
//     [sim, u__] = ld_gain(sim, ev, tmp, Ta/2);
    
    [sim,z_fb] = libdyn_new_feedback(sim);
    
  [sim, sum_] = ld_sum(sim, ev, list(u__, z_fb), 1, 1);
  [sim, y] = ld_sat(sim, ev, sum_, min__, max__);

  [sim, y_] = ld_ztf(sim, ev, y, 1/z);
    
    [sim] = libdyn_close_loop(sim, y_, z_fb);    
endfunction

function [sim, y] = ld_limited_integrator4(sim, ev, u, min__, max__, Ta) // PARSEDOCU_BLOCK
// %PURPOSE: Implements a time discrete integrator with saturation of the output between min__ and max__
// compared to ld_limited_integrator there is no delay: Ta z / (z-1)
//
// u * - input
// y * - output
// min__ * - variable saturation minimum
// 
// y(k+1) = sat( y(k) + Ta*u , min__, max__ )


    [sim, u__] = ld_gain(sim, ev, u, Ta);
    
    [sim,z_fb] = libdyn_new_feedback(sim);
    
  [sim, sum_] = ld_sum(sim, ev, list(u__, z_fb), 1, 1);
  [sim, y_sat] = ld_sat(sim, ev, sum_, 0, max__);
  
  [sim, y_missing] = ld_add(sim, ev, list(min__, y_sat), [1,-1]);
  [sim, is_missing] = ld_compare_01(sim, ev, in=y_missing,  thr=0);
  [sim, y_add] = ld_mult(sim, ev, inp_list=list(y_missing, is_missing), muldiv1_list=[0, 0]);
  [sim, y] = ld_add(sim, ev, list(y_sat, y_add), [1,1]);

  [sim, y_] = ld_ztf(sim, ev, y, 1/z);
    
    [sim] = libdyn_close_loop(sim, y_, z_fb);    

endfunction

function [sim, u] = ld_lin_awup_controller(sim, ev, r, y, Ta, tfR, min__, max__) // PARSEDOCU_BLOCK
// %PURPOSE: linear controller with anti reset windup implemented by bounding the integral state:
// e = r-y
// u = ld_limited_integrator( e, min__, max__ ) + tfR*e
    [sim, e] = ld_sum(sim, ev, list(r, y), 1, -1);
    
    [sim,u1] = ld_limited_integrator(sim, ev, e, min__, max__, Ta);
    [sim,u2] = ld_ztf(sim, ev, e, tfR);
    
    [sim,u] = ld_sum(sim, ev, list(u1,u2), 1,1);
endfunction


function [sim] = ld_print_angle(sim, ev, alpha, text) // PARSEDOCU_BLOCK
// %PURPOSE: Convert an angle in rad to degree and print to console
// 
// alpha * - angle signal
// text - string
// 
    [sim, alpha_deg] = ld_gain(sim, ev, alpha, 1/%pi*180);
    [sim] = ld_printf(sim, ev, alpha_deg, text, 1);
endfunction

function [sim, pwm] = ld_pwm(sim, ev, plen, u) // PARSEDOCU_BLOCK
// 
// %PURPOSE: PWM generator
// 
// plen - period length
// u * - modulation signal; Values are between 0 and 1.
// pwm * - pwm output
//

    [sim,u] = ld_gain(sim, ev, u, plen);
    
    [sim,one] = ld_const(sim, ev, 1);
    
    [sim,modcount] = ld_modcounter(sim, ev, in=one, initial_count=0, mod=plen);
    
    [sim, test] = ld_add(sim, ev, list(modcount, u), [-1,1] );
    [sim,pwm] = ld_compare_01(sim, ev, test,  thr=0);
endfunction

function [sim, outvec, Nvecplay] = ld_vector_play(sim, ev, A, special) // PARSEDOCU_BLOCK
// 
// %PURPOSE: Play a vectorial signal
// 
// A - matrix containing the vectors to play
// outvec*+(length(A(:,1))) - 
// Nvecplay = length of output signal vector
//
// outputs A(:,i), where i is increasing within each time step
// special = [ "repeate" ]
//

  [Nvecplay,Nsamples] = size(A); // m is the number of samples in time

  data = A(:); Ndata = length(data);

  // create a new vector
  [sim,vector] = ld_constvec(sim, ev, data );
  
  // vector extract test
  
  if special == "repeate" then
    [sim,one] = ld_const(sim, ev, 1);
    [sim, index] = ld_modcounter(sim, ev, in=one, initial_count=0, mod=Nsamples);
  else
    error("wrong special string. Should be one of ""repeate"", ...");
  end

//  [sim] = ld_printf(sim, ev, index, "index = ", 1);

  [sim, start_at] = ld_gain(sim, ev, index, Nvecplay);
  [sim, start_at] = ld_add_ofs(sim, ev, start_at, 1);

  [sim,outvec] = ld_vector_extract(sim, ev, in=vector, from=start_at, window_len=Nvecplay, vecsize=Ndata );
endfunction


function [sim,y] = ld_alternate( sim, ev, start_with_zero ) // PARSEDOCU_BLOCK
//
// %PURPOSE: generate an alternating sequence     
//
// y * - output
//
//
// [0, 1, 0, 1, 0, ... ], if start_with_zero == %T
// [1, 0, 1, 0, 1, ... ], if start_with_zero == %F
//
  z = poly(0,'z');    

  [sim,one] = ld_const(sim, ev, 1);

  [sim, fb] = libdyn_new_feedback();
  
  [sim, su ] = ld_add(sim, ev, list(fb, one), [-1,1] );
  [sim, del] = ld_ztf(sim, ev, su, 1/z); // a delay of one sample
  
  [sim] = libdyn_close_loop(sim, del, fb);
  
  y = del; 
  
  if (start_with_zero == %F) then
    [sim, y] = ld_not(sim, ev, y);
  end
    
endfunction

function [sim, out] = ld_detect_step_event(sim, ev, in, eps) // PARSEDOCU_BLOCK
    //
    // %PURPOSE: step detection block
    //
    // Detect jumps in the signal "in".
    // Everytime a jump occurs "out" is an impulse with the intensity of the
    // value after the jump i.e. if a signal steps from 1 to 2 there
    // will be an impulse out = 2
    // if no steps occur, out is zero
    //
    
    z = poly(0, 'z');
    
    [sim, i1] = ld_ztf(sim, ev, in, (z-1)/z ); // diff
    [sim, i2] = ld_abs(sim, ev, i1);
    
    [sim,event] = ld_compare_01(sim, ev, in=i2, thr=eps);
    [sim, out] = ld_mult(sim, ev, inp_list=list(event, in), muldiv1_list=[0, 0]);
    
endfunction

function [sim, out] = ld_detect_step_event2(sim, ev, in, eps) // PARSEDOCU_BLOCK
    //
    // %PURPOSE: step detection block
    //
    // Detect jumps in the signal "in".
    // Everytime a jump occurs "out" is an impulse with an intensity of 1,
    // else it is zero
    //
    
    z = poly(0, 'z');
    
    [sim, i1] = ld_ztf(sim, ev, in, (z-1)/z ); // diff
    [sim, i2] = ld_abs(sim, ev, i1);
    
    [sim,event] = ld_compare_01(sim, ev, in=i2, thr=eps);
    out = event;
    
endfunction

function [sim, reached] = ld_reference_reached(sim, ev, r, y, N, eps) // PARSEDOCU_BLOCK
    //
    // %PURPOSE: check wheter a reference value is reached 
    //
    // r * - Reference Signal (shall be constant)
    // y * - Signal to compare to the reference
    // N - After the condition of reaching the reference is true
    //     N time steps will be wait until setting reached to one. 
    // eps - the half tolerance band width
    //

  [sim, e] = ld_add(sim, ev, list(r,y), [1,-1] );
  //[sim, i1] = ld_ztf(sim, ev, e, 1/(3+1) * (1 + z^(-1) + z^(-2) + z^(-3) ) );

  i1 = e;

  [sim, i3] = ld_abs(sim, ev, i1);
  [sim, reached] = ld_belowEpsForNSteps(sim, ev, in=i3,  thr=eps, N);

endfunction

function [sim, reached] = ld_greaterEpsForNSteps(sim, ev, in, thr, N) // PARSEDOCU_BLOCK
    //
    // %PURPOSE: return true, when input is greater a constant for more than N sampling steps
    //
    // in * - input signal
    // thr - threshold constant
    // N - integer
    //
    // If in is greater than thr for more than N time steps, reached = 1;
    // Otherwise reached is set to 0.
    
  [sim, i4] = ld_compare_01(sim, ev, in,  thr);
  [sim, i5] = ld_not(sim, ev, in=i4);
  
  [sim, resetto] = ld_const(sim, ev, 0);
  [sim, count] = ld_counter(sim, ev, count=i4, reset=i5, resetto, initial=0);

  [sim, reached] = ld_compare_01(sim, ev, in=count,  thr=N);
    
endfunction

function [sim, reached] = ld_belowEpsForNSteps(sim, ev, in, thr, N) // PARSEDOCU_BLOCK
    //
    // %PURPOSE: return true, when input is below a constant for more than N sampling steps
    //
    // in * - input signal
    // thr - threshold constant
    // N - integer
    //
    // If in is below thr for more than N time steps, reached = 1;
    // Otherwise reached is set to 0.
    
  [sim, i4] = ld_compare_01(sim, ev, in,  thr);
  [sim, i5] = ld_not(sim, ev, in=i4);
  
  [sim, resetto] = ld_const(sim, ev, 0);
  [sim, count] = ld_counter(sim, ev, count=i5, reset=i4, resetto, initial=0);

  [sim, reached] = ld_compare_01(sim, ev, in=count,  thr=N);
    
endfunction


function [sim, reached, count, countup, reset] = ld_belowEpsForNStepsAdvanced(sim, ev, in, thr, N) // PARSEDOCU_BLOCK
    //
    // %PURPOSE: return true, when input is below a constant for more than N sampling steps
    //
    // in * - input signal
    // thr - threshold constant
    // N - integer
    //
    // If in is below thr for N time steps, reached = 1;
    // Otherwise reached is set to 0.
    
  [sim, reset] = ld_compare_01(sim, ev, in,  thr);
  [sim, countup] = ld_not(sim, ev, in=reset);
  
  [sim, resetto] = ld_const(sim, ev, 0);
  [sim, count] = ld_counter(sim, ev, countup, reset, resetto, initial=0);

  [sim, reached] = ld_compare_01(sim, ev, in=count,  thr=N-1);
    
endfunction

function [sim] = ld_file_save_machine(sim, ev, in, cntrl, intype, insize, fname) // PARSEDOCU_BLOCK
//
// %PURPOSE: Start and stop saving of data to files
// 
//   in *+(size) - Data to write
//   cntrl * - if cntrl steps to 2 then saving is started; if it steps to 1 saving is stopped
//   intype - ORTD input type of data
//   size - amount of elements in the vector in
//   fname - string: Filename for saving
// 
// Note: The implementation of this function is a superblock using state machines
//       and the ld_savefile block. If you're interested take the source as an example.
// 
// 

    function [sim, outlist, active_state, x_global_kp1, userdata] = state_mainfn(sim, inlist, x_global, state, statename, userdata)
      // This function is called multiple times -- once for each state.
      // At runtime, these are three different nested simulations. Switching
      // between them represents state changing, thus each simulation 
      // represents a certain state.
      
//       printf("defining savemachine state %s (#%d) ... userdata(1)=%s\n", statename, state, userdata(1) );
      
      // define names for the first event in the simulation
      ev = 0; events = ev;

      // 
      dataToSave = inlist(1);
      switch = inlist(2);  [sim, switch] = ld_gain(sim, ev, switch, 1);


      // demultiplex x_global
      [sim, x_global] = ld_demux(sim, events, vecsize=1, invec=x_global);


      // The signals "active_state" is used to indicate state switching: A value > 0 means the 
      // the state enumed by "active_state" shall be activated in the next time step.
      // A value less or equal to zero causes the statemachine to stay in its currently active
      // state

      select state
  case 1 // state 1
    active_state = switch;
    [sim] = ld_printf(sim, ev, in=dataToSave, str="Pauseing Save", insize=DataLen);

  case 2 // state 2
    [sim] = ld_savefile(sim, ev, fname, source=dataToSave, vlen=DataLen);
    [sim] = ld_printf(sim, ev, in=dataToSave, str="Saveing", insize=DataLen);

    active_state = switch;
      end

      // multiplex the new global states
      [sim, x_global_kp1] = ld_mux(sim, ev, vecsize=1, inlist=x_global);
      
      // the user defined output signals of this nested simulation
      outlist = list();
  endfunction

  DataLen = insize;


//   [sim] = ld_printf(sim, ev, cntrl, "cntrl", 1);

  [sim, cntrl] = ld_detect_step_event(sim, ev, in=cntrl, eps=0.2);

//   [sim] = ld_printf(sim, ev, cntrl, "Dcntrl", 1);


  // set-up two states represented by two nested simulations
  [sim, outlist, x_global, active_state,userdata] = ld_statemachine(sim, ev=0, ...
      inlist=list(in, cntrl), ..
      insizes=[insize,1], outsizes=[], ... 
      intypes=[intype ,ORTD.DATATYPE_FLOAT  ], outtypes=[], ...
      nested_fn=state_mainfn, Nstates=2, state_names_list=list("pause", "save"), ...
      inittial_state=1, x0_global=[1], userdata=list("hallo")  );


endfunction







// 
// Blocks, which C functions have not been move to the basic module yet, but the interfacing function
// 



// compare block. If input > thr: 
// optional_cmp_mode342 == 0: output = 1; else -1
// optional_cmp_mode342 == 1: output = 1; else 0
function [sim,bid] = libdyn_new_compare(sim, events, thr, optional_cmp_mode342)    
  if (exists('optional_cmp_mode342') ~= 1) then
    optional_cmp_mode342 = 0;
  end

  btype = 140;
  [sim,bid] = libdyn_new_blk_generic(sim, events, btype, [optional_cmp_mode342], [thr]);
endfunction
function [sim,y] = ld_compare(sim, events, in,  thr) // PARSEDOCU_BLOCK
//
// %PURPOSE: compare block. 
//   thr - constant
//   in * - signal
//   y *
// If in > thr: y = 1; else y = -1
// 
// Please note: returns -1 for in == 0
//

    [sim,blk] = libdyn_new_compare(sim, events, thr);
    [sim,blk] = libdyn_conn_equation(sim, blk, list(in,0));
    [sim,y] = libdyn_new_oport_hint(sim, blk, 0);    
endfunction
function [sim,y] = ld_compare_01(sim, events, in,  thr) // PARSEDOCU_BLOCK
//
// %PURPOSE: compare block. 
//   thr - constant
//   in - signal
//   y *
// If in > thr: y = 1; else y = 0
//

    [sim,blk] = libdyn_new_compare(sim, events, thr, 1); // mode = 1
    [sim,blk] = libdyn_conn_equation(sim, blk, list(in,0));
    [sim,y] = libdyn_new_oport_hint(sim, blk, 0);    
endfunction


















//
// Blocks
//


// FIXME: komische Funktion (noch gebraucht?)
function [sim,bid] = libdyn_new_blk_sum_pn(sim)
  btype = 10;
  [sim,oid] = libdyn_new_objectid(sim);
  id = oid; // id for this parameter set
  
  sim.parlist = new_irparam_elemet(sim.parlist, id, IRPAR_LIBDYN_BLOCK, [btype; oid], []);
endfunction

// FIXME: komische Funktion (noch gebraucht?)
function [sim,bid] = libdyn_new_blk_gen(sim, events, symbol_name, ipar, rpar)
  btype = 5000;
  str = str2code(symbol_name);
  
  [sim,bid] = libdyn_new_blk_generic(sim, events, btype, [length(str); str; ipar], [rpar]);
endfunction











// 
// 

// // function [sim,bid] = libdyn_new_blk_fngen(sim, events, shape_)
// //   btype = 80;
// //   [sim,bid] = libdyn_new_blk_generic(sim, events, btype, [shape_], []);
// // endfunction
// function [sim,out] = ld_fngen(sim, events, shape_) // PARSEDOCU_BLOCK
// //
// // %PURPOSE: function generator
// // 
// // shape_ - the shape of the output signal: =0 : ???
// // out * - output
// // 
// //
// 
//   ortd_checkpar(sim, list('SingleValue', 'shape_', shape_) );
// 
//   btype = 80;
//   [sim,bid] = libdyn_new_block(sim, events, btype, [shape_], [], ...
//                    insizes=[], outsizes=[1], ...
//                    intypes=[], outtypes=[ORTD.DATATYPE_FLOAT]  );
// 
//   [sim,out] = libdyn_conn_equation(sim, bid, list());
//   [sim,out] = libdyn_new_oport_hint(sim, out, 0);
// endfunction






 
// serial to parallel
function [sim,bid] = libdyn_new_blk_s2p(sim, events, len)
  btype = 90;
  [sim,bid] = libdyn_new_blk_generic(sim, events, btype, [len], []);
endfunction



//
// A swich with two inputs and one output depending on a state. initial state=initial_state
// state 0 means the first input is feed through
// state 1 means the first input is feed through
//
// Switching occurs to state 0 if event 1 occurs
// Switching occurs to state 1 if event 2 occurs
// event 0 is the normal regular event
//

function [sim,bid] = libdyn_new_blk_2to1evsw(sim, events, initial_state)
  btype = 110;
  [sim,bid] = libdyn_new_blk_generic(sim, events, btype, [initial_state], []);
endfunction














//////////////////////////////////////////////////////////
// fancy shortcuts, that *should* be used by the user
//////////////////////////////////////////////////////////

function [sim,bid] = libdyn_new_blk_const(sim, events, c1)
  btype = 40;
  [sim,bid] = libdyn_new_block(sim, events, btype, [], [c1], ...
                   insizes=[], outsizes=[1], ...
                   intypes=[], outtypes=[ORTD.DATATYPE_FLOAT]  );
endfunction
function [sim,c] = ld_const(sim, events, val) // PARSEDOCU_BLOCK
//
// %PURPOSE: A constant val
//

    [sim,c] = libdyn_new_blk_const(sim, events, val); // Instead of event a predefined initial event that only occurs once should be used
    [sim,c] = libdyn_new_oport_hint(sim, c, 0);    
endfunction



// FIXME no port size checking
function [sim,bid] = libdyn_new_blk_sum(sim, events, c1, c2)
  btype = 12;
  
  [sim,bid] = libdyn_new_block(sim, events, btype, [], [c1; c2], ...
                   insizes=[1,1], outsizes=[1], ...
                   intypes=[ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_FLOAT], outtypes=[ORTD.DATATYPE_FLOAT]);
  
//  [sim,bid] = libdyn_new_blockid(sim);
//  id = bid; // id for this parameter set
//  
//  sim.parlist = new_irparam_elemet(sim.parlist, id, IRPAR_LIBDYN_BLOCK, [btype; bid], [c1; c2]);
endfunction
function [sim,sum_] = ld_sum(sim, events, inp_list, fak1, fak2) 
// FIXME obsolete
    [sim,sum_] = libdyn_new_blk_sum(sim, events, fak1, fak2);
    [sim,sum_] = libdyn_conn_equation(sim, sum_, inp_list);  
    [sim,sum_] = libdyn_new_oport_hint(sim, sum_, 0);    
endfunction
function [sim,sum_] = ld_add(sim, events, inp_list, fak_list) // PARSEDOCU_BLOCK
//
// %PURPOSE: Add signals (linear combination)
// inp_list = list( in1, in2 )  ; fak_list = [ c1, c2 ]
// sum_ = in1 * c1 + in2 * c2
//

  ortd_checkpar(sim, list('SignalList', 'inp_list', inp_list) );
  ortd_checkpar(sim, list('Vector', 'fak_list', fak_list) );


    [sim,sum_] = libdyn_new_blk_sum(sim, events, fak_list(1), fak_list(2));
    [sim,sum_] = libdyn_conn_equation(sim, sum_, inp_list);  
    [sim,sum_] = libdyn_new_oport_hint(sim, sum_, 0);    
endfunction





// Multiplication 
// d1, d2: multiplicate (=0) or divide (=1) corresponding input; need exactly 2 inputs
function [sim,bid] = libdyn_new_blk_mul(sim, events, d1, d2)
  btype = 70;
  [sim,bid] = libdyn_new_block(sim, events, btype, [d1, d2], [], ...
                   insizes=[1,1], outsizes=[1], ...
                   intypes=[ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_FLOAT], outtypes=[ORTD.DATATYPE_FLOAT]);
endfunction
function [sim,mul_] = ld_mul(sim, events, inp_list, muldiv1, muldiv2)
// %PURPOSE: Multiplication 
// muldiv1/2: multiplicate (=0) or divide (=1) corresponding input; need exactly 2 inputs
 // FIXME obsolete
    [sim,mul_] = libdyn_new_blk_mul(sim, events, muldiv1, muldiv2);
    [sim,mul_] = libdyn_conn_equation(sim, mul_, inp_list);  
    [sim,mul_] = libdyn_new_oport_hint(sim, mul_, 0);    
endfunction
function [sim,mul_] = ld_dot(sim, events, inp_list, muldiv1_list)
// %PURPOSE: Multiplication 
// muldiv1/2: multiplicate (=0) or divide (=1) corresponding input; need exactly 2 inputs
// inp_list = list( in1, in2 )  ; muldiv1_list = [ muldiv1, muldiv2 ]
    [sim,mul_] = libdyn_new_blk_mul(sim, events, muldiv1_list(1), muldiv1_list(2) );
    [sim,mul_] = libdyn_conn_equation(sim, mul_, inp_list);  
    [sim,mul_] = libdyn_new_oport_hint(sim, mul_, 0);    
endfunction
function [sim,mul_] = ld_mult(sim, events, inp_list, muldiv1_list) // PARSEDOCU_BLOCK
//
// %PURPOSE: Multiplication 
// muldiv1/2: multiplicate (=0) or divide (=1) corresponding input; need exactly 2 inputs
// inp_list = list( in1, in2 )  ; muldiv1_list = [ muldiv1, muldiv2 ]
//
    [sim,mul_] = libdyn_new_blk_mul(sim, events, muldiv1_list(1), muldiv1_list(2) );
    [sim,mul_] = libdyn_conn_equation(sim, mul_, inp_list);  
    [sim,mul_] = libdyn_new_oport_hint(sim, mul_, 0);    
endfunction











function [sim,out] = ld_gain(sim, events, in, gain) // PARSEDOCU_BLOCK
//
// %PURPOSE: A simple gain
// 
// in * - input
// out * - output
// 
// out = in * gain
//

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'in', in) );
  ortd_checkpar(sim, list('SingleValue', 'gain', gain) );
end

  [inp] = libdyn_extrakt_obj(in ); // compatibility

  btype = 20;
  [sim,bid] = libdyn_new_block(sim, events, btype, [], [gain], ...
                   insizes=[1], outsizes=[1], ...
                   intypes=[ORTD.DATATYPE_FLOAT], outtypes=[ORTD.DATATYPE_FLOAT]  );

  [sim,out] = libdyn_conn_equation(sim, bid, list(inp,0));
  [sim,out] = libdyn_new_oport_hint(sim, out, 0);    
endfunction








function [sim,sign_] = ld_sign(sim, events, inp_list, thr) // PARSEDOCU_BLOCK
//
// FIXME: OSOLETE FN: use ld_getsign instead
// 
// return the sign of the input sigal
// either 1 or -1
// 
// 
//
    [inp] = libdyn_extrakt_obj( inp_list ); // compatibility

    [sim,sign_] = libdyn_new_compare(sim, events, thr);
    [sim,sign_] = libdyn_conn_equation(sim, sign_, list(inp));
    [sim,sign_] = libdyn_new_oport_hint(sim, sign_, 0);
endfunction











function [sim,lkup] = ld_lkup(sim, events, inp_list, lower_b, upper_b, table)
// %PURPOSE: lookup table
//
// inp_list - input signal
// table - a vector of values to look up
// the input is mapped between lower_b and upper_b to the
// index within table
  [inp] = libdyn_extrakt_obj( inp_list ); // compatibility

    [sim,lkup] = libdyn_new_blk_lkup(sim, events, lower_b, upper_b, table);
    [sim,lkup] = libdyn_conn_equation(sim, lkup, list(inp));
    [sim,lkup] = libdyn_new_oport_hint(sim, lkup, 0);
endfunction
  
  
  
  
  
  
  
function [sim,out] = ld_fngen(sim, events, shape_, period, amp) // PARSEDOCU_BLOCK
//
// %PURPOSE: function generator
// 
// shape_ - the shape of the output signal: =0 : sinus, more to come...
// period, amp * - Periode length in samples and amplitude
// out * - output
// 
//

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('SingleValue', 'shape_', shape_) );
end

  btype = 80;
  [sim,bid] = libdyn_new_block(sim, events, btype, [shape_], [], ...
                   insizes=[1,1], outsizes=[1], ...
                   intypes=[ORTD.DATATYPE_FLOAT,ORTD.DATATYPE_FLOAT], outtypes=[ORTD.DATATYPE_FLOAT]  );

  [sim,out] = libdyn_conn_equation(sim, bid, list(period, amp));
  [sim,out] = libdyn_new_oport_hint(sim, out, 0);
endfunction

  
  


// function [sim,delay] = ld_delay(sim, events, inp_list, delay_len)
//     [inp] = libdyn_extrakt_obj( inp_list ); // compatibility
// 
//     [sim,delay] = libdyn_new_delay(sim, events, delay_len)
//     [sim,delay] = libdyn_conn_equation(sim, delay, list(inp));
//     [sim,delay] = libdyn_new_oport_hint(sim, delay, 0);    
// endfunction












// FIXME no port size checking
function [sim,bid] = libdyn_new_blk_zTF(sim, events, H)
  btype = 30;
  bip = [ degree(H.num); degree(H.den) ];
  brp = [ coeff(H.num)'; coeff(H.den)' ];

  [sim,bid] = libdyn_new_block(sim, events, btype, [bip], [brp], ...
                   insizes=[1], outsizes=[1], ...
                   intypes=[ORTD.DATATYPE_FLOAT], outtypes=[ORTD.DATATYPE_FLOAT]);
  
//  [sim,bid] = libdyn_new_blockid(sim);
//  id = bid; // id for this parameter set
//  
//  
//  sim.parlist = new_irparam_elemet(sim.parlist, id, IRPAR_LIBDYN_BLOCK, [btype; bid; bip], [brp]);
endfunction
function [sim,y] = ld_ztf(sim, events, inp_list, H) // PARSEDOCU_BLOCK
//
// %PURPOSE: Time discrete transfer function
// H is give as a Scilab rational
//

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'inp_list', inp_list) );
end

  [inp] = libdyn_extrakt_obj( inp_list ); // compatibility

    [sim,tf] = libdyn_new_blk_zTF(sim, events, H);
    [sim,y] = libdyn_conn_equation(sim, tf, list(inp));
    [sim,y] = libdyn_new_oport_hint(sim, y, 0);    
endfunction










function [sim,bid] = libdyn_new_blk_sat(sim, events, lowerlimit, upperlimit)
  btype = 50;
  [sim,bid] = libdyn_new_blk_generic(sim, events, btype, [], [lowerlimit, upperlimit], ...
                   insizes=[1], outsizes=[1], ...
                   intypes=[ORTD.DATATYPE_FLOAT], outtypes=[ORTD.DATATYPE_FLOAT]);
endfunction
function [sim,y] = ld_sat(sim, events, inp_list, lowerlimit, upperlimit) // PARSEDOCU_BLOCK
//
// %PURPOSE: Saturation between lowerlimit and upperlimit
//


  [inp] = libdyn_extrakt_obj( inp_list ); // compatibility

    [sim,sat] = libdyn_new_blk_sat(sim, events, lowerlimit, upperlimit);
    [sim,y] = libdyn_conn_equation(sim, sat, list(inp));
    [sim,y] = libdyn_new_oport_hint(sim, y, 0);    
endfunction










function [sim,bid] = libdyn_new_flipflop(sim, events, initial_state)
  btype = 160;
  [sim,bid] = libdyn_new_block(sim, events, btype, [initial_state], [], ...
                   insizes=[1,1,1], outsizes=[1], ...
                   intypes=[ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_FLOAT], outtypes=[ORTD.DATATYPE_FLOAT]);
endfunction
function [sim,y] = ld_flipflop(sim, events, set0, set1, reset, initial_state) // PARSEDOCU_BLOCK
//
// %PURPOSE: A flip-flop
//
// set0, set1, reset * - control of the flipflop (set output to zero if set0 is >0.5 for at least one sample, ...)
// initial_state - constant
//

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'set0', set0) );
  ortd_checkpar(sim, list('Signal', 'set1', set1) );
  ortd_checkpar(sim, list('Signal', 'reset', reset) );
  ortd_checkpar(sim, list('SingleValue', 'init_state', init_state) );
end

    [sim,blk] = libdyn_new_flipflop(sim, events, initial_state);
    [sim,blk] = libdyn_conn_equation(sim, blk, list(set0,0, set1,0, reset,0)); // FIXME: remove ,0
    [sim,y] = libdyn_new_oport_hint(sim, blk, 0);    
endfunction




// function [sim,bid] = libdyn_new_printf(sim, events, str, insize) //REMOVE
//   btype = 170;
//   str = ascii(str);
//   [sim,bid] = libdyn_new_blk_generic(sim, events, btype, [insize, length(str), str(:)'], []);
// endfunction



//
// Some terminal color codes for usage with ld_printf  MOVED TO libdyn.sci as a workaround
//

// ORTD.termcode.red = ascii(27) + '[31m';
// ORTD.termcode.green = ascii(27) + '[32m';
// ORTD.termcode.yellow = ascii(27) + '[33m';
// ORTD.termcode.blue = ascii(27) + '[34m';
// ORTD.termcode.reset = ascii(27) + '[0m';


function [sim] = ld_printf(sim, events, in, str, insize) // PARSEDOCU_BLOCK
//
// %PURPOSE: Print data to stdout (the console)
//
// in *+(insize) - vectorial input signal
//
// str is a string that is printed followed by the signal vector in
// of size insize
//
// Hint: Apply colored printf's by using the predefined terminal color codes:
// 
// str = ORTD.termcode.red + "some colored text..." + ORTD.termcode.reset
// 
// instead of red there currently is: green, yellow, blue.
// 
// 
// 

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'in', in) );
  ortd_checkpar(sim, list('SingleValue', 'insize', insize) );
  ortd_checkpar(sim, list('String', 'str', str) );
end

  btype = 170;
  str = ascii(str);
//   [sim,blk] = libdyn_new_blk_generic(sim, events, btype, [insize, length(str), str(:)'], []);

  [sim,blk] = libdyn_new_block(sim, events, btype, ipar=[ insize, length(str), str(:)' ], rpar=[ ], ...
                   insizes=[ insize ], outsizes=[], ...
                   intypes=[ ORTD.DATATYPE_FLOAT ], outtypes=[]  );


  [sim,blk] = libdyn_conn_equation(sim, blk, list(in,0) );
endfunction

function [sim] = ld_printfInt32(sim, events, in, str, insize) // PARSEDOCU_BLOCK
//
// %PURPOSE: Print data to stdout (the console)
//
// in *+(insize) - vectorial input signal (Int32)
//
// str is a string that is printed followed by the signal vector in
// of size insize
//
// Hint: Apply colored printf's by using the predefined terminal color codes:
// 
// str = ORTD.termcode.red + "some colored text..." + ORTD.termcode.reset
// 
// instead of red there currently is: green, yellow, blue.
// 
// 
// 

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'in', in) );
  ortd_checkpar(sim, list('SingleValue', 'insize', insize) );
  ortd_checkpar(sim, list('String', 'str', str) );
end

  btype = 60001 + 78;
  str = ascii(str);
//   [sim,blk] = libdyn_new_blk_generic(sim, events, btype, [insize, length(str), str(:)'], []);

  [sim,blk] = libdyn_new_block(sim, events, btype, ipar=[ insize, length(str), str(:)' ], rpar=[ ], ...
                   insizes=[ insize ], outsizes=[], ...
                   intypes=[ ORTD.DATATYPE_INT32 ], outtypes=[]  );


  [sim,blk] = libdyn_conn_equation(sim, blk, list(in,0) );
endfunction

function [sim] = ld_printfBin(sim, events, in, str, insize) // PARSEDOCU_BLOCK
//
// %PURPOSE: Print data to stdout (the console)
//
// in *+(insize) - vectorial input signal (Binaray)
//
// str is a string that is printed followed by the signal vector in
// of size insize
//
// Hint: Apply colored printf's by using the predefined terminal color codes:
// 
// str = ORTD.termcode.red + "some colored text..." + ORTD.termcode.reset
// 
// instead of red there currently is: green, yellow, blue.
// 
// 
// 

if ORTD.FASTCOMPILE==%f then
  ortd_checkpar(sim, list('Signal', 'in', in) );
  ortd_checkpar(sim, list('SingleValue', 'insize', insize) );
  ortd_checkpar(sim, list('String', 'str', str) );
end

  btype = 60001 + 93;
  str = ascii(str);
//   [sim,blk] = libdyn_new_blk_generic(sim, events, btype, [insize, length(str), str(:)'], []);

  [sim,blk] = libdyn_new_block(sim, events, btype, ipar=[ insize, length(str), str(:)' ], rpar=[ ], ...
                   insizes=[ insize ], outsizes=[], ...
                   intypes=[ ORTD.DATATYPE_BINARY ], outtypes=[]  );


  [sim,blk] = libdyn_conn_equation(sim, blk, list(in,0) );
endfunction



//
// A switching Block
// inputs = [control_in, signal_in]
// if control_in > 0 : signal_in is directed to output 1; output_2 is set to zero
// if control_in < 0 : signal_in is directed to output 2; output_1 is set to zero
//
function [sim,bid] = libdyn_new_blk_switch(sim, events)
  btype = 60;
  [sim,bid] = libdyn_new_block(sim, events, btype, [], [], ...
                   insizes=[1,1], outsizes=[1,1], ...
                   intypes=[ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_FLOAT], outtypes=[ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_FLOAT]);
endfunction
function [sim,out_1, out_2] = ld_switch(sim, events, cntrl, in) // PARSEDOCU_BLOCK
//
// %PURPOSE: A switching Block
// inputs = [control_in, signal_in]
// if control_in > 0 : signal_in is directed to output 1; output_2 is set to zero
// if control_in < 0 : signal_in is directed to output 2; output_1 is set to zero
//
    [sim,blk] = libdyn_new_blk_switch(sim, events);
    [sim,blk] = libdyn_conn_equation(sim, blk, list(cntrl,0, in,0));
    
    [sim,out_1] = libdyn_new_oport_hint(sim, blk, 0);
    [sim,out_2] = libdyn_new_oport_hint(sim, blk, 1);
endfunction










//
// Sample play block
//
// plays the sequence stored in r
// each time event 0 occurs the next value of r is put out
// sampling start either imediadedly (initial_play=1) or on event 1.
// Event 2 stops sampling and set ouput to last values (mute_afterstop = 0 and hold_last_values == 1) 
// or zero (mute_afterstop = 1)
//
function [sim,bid] = libdyn_new_blk_play(sim, events, r, initial_play, hold_last_value, mute_afterstop)
  if (exists('initial_play') ~= 1) then
    initial_play = 1;  
  end
  if (exists('hold_last_value') ~= 1) then
    hold_last_value = 0;  
  end
  if (exists('mute_afterstop') ~= 1) then
    mute_afterstop = 0;  
  end
  
  btype = 100;
  [sim,bid] = libdyn_new_block(sim, events, btype, [length(r), initial_play, hold_last_value, mute_afterstop], [r], ...
                   insizes=[], outsizes=[1], ...
                   intypes=[], outtypes=[ORTD.DATATYPE_FLOAT]);
endfunction
function [sim,y] = ld_play_simple(sim, events, r) // PARSEDOCU_BLOCK
//
// %PURPOSE: Simple sample play block
//
// plays the sequence stored in r
// each time event 0 occurs the next value of r is put out
//

  [sim,y] = libdyn_new_blk_play(sim, events, r, 1, 1, 0);
  [sim,y] = libdyn_new_oport_hint(sim, y, 0);    
endfunction










// Dump at max maxlen samples to file "filename"; start automatically if autostart == 1
function [sim,bid] = libdyn_new_blk_filedump(sim, events, filename, vlen, maxlen, autostart)
  btype = 130;
  fname = ascii(filename);
  [sim,bid] = libdyn_new_block(sim, events, btype, [maxlen, autostart, vlen, length(fname), fname(:)'], [], ...
                   insizes=[vlen], outsizes=[], ...
                   intypes=[ORTD.DATATYPE_FLOAT], outtypes=[]);
endfunction
function [sim,save_]=libdyn_dumptoiofile(sim, events, fname, source) //OBSOLET
// %PURPOSE: Quick and easy dumping of signals to files in one line of code
// obsolete version
  [source] = libdyn_extrakt_obj( source ); // compatibility

  // source: a list with a block + a port
  [sim,save_] = libdyn_new_blk_filedump(sim, events, fname, 1, 0, 1);
  [sim,save_] = libdyn_conn_equation(sim, save_, list(source, 0) );
endfunction
function [sim,save_]=ld_dumptoiofile(sim, events, fname, source) //OBSOLET
//
// Quick and easy dumping of signals to files
// source - signal of size 1 (at the moment)
// fname - filename string
//
  [inp] = libdyn_extrakt_obj( source ); // compatibility

  // source: a list with a block + a port
  [sim,save_] = libdyn_new_blk_filedump(sim, events, fname, 1, 0, 1);
  [sim,save_] = libdyn_conn_equation(sim, save_, list(source) );
endfunction




//////////////////////////////////////////////////////////
// More complex blocks based on elementary blocks
//////////////////////////////////////////////////////////

//
// Generic controllers
//

function [sim,u] = ld_standard_controller(sim, event, r, y, K)
// classic linear controller
    [sim,e] = ld_sum(sim, event, list(r,0, y,0), 1,-1 );
    [sim,u] = ld_ztf(sim, event, list(e,0), K);
endfunction

function [sim,u] = ld_standard_controller_2dof(sim, event, r, y, K, M)
// like classic controller but with a measurement filter M
    [sim,y_] = ld_ztf(sim, event, list(y,0), M);
    [sim,e] = ld_sum(sim, event, list(r,0, y_,0), 1,-1 );
    [sim,u] = ld_ztf(sim, event, list(e,0), K);
endfunction

function [sim,u,u_fb,u_ff] = ld_standard_controller_ffw(sim, event, r, y, K, Gm1, T)
// controller with a feedforwad part
    [sim,r_] = ld_ztf(sim, event, list(r,0), T); // closed loop model
    
    [sim,e] = ld_sum(sim, event, list(r_,0, y,0), 1,-1 );
    [sim,u_fb] = ld_ztf(sim, event, list(e,0), K);
    
    [sim,u_ff] = ld_ztf(sim, event, list(r,0), Gm1);
    [sim,u] = ld_sum(sim, event, list(u_fb,0, u_ff,0), 1,1 );
endfunction



function [sim] = ld_EDFWrite(sim, events, fname, smp_freq, inlist, innames, PhyMin, PhyMax, DigMin, DigMax) // PARSEDOCU_BLOCK
// Write data to EDF+ - Files
//
// inlist - list() of signals of size 1
// 
// 


  Nin = length(inlist);

  // Nr of channels
  chns = Nin;

  if chns ~= length(innames) then
    error("chns ~= length(innames)");
  end
  if chns ~= length(PhyMax) then
    error("chns ~= length(PhyMax)");
  end
  if chns ~= length(PhyMin) then
    error("chns ~= length(PhyMin)");
  end
  if chns ~= length(DigMax) then
    error("chns ~= length(DigMax)");
  end
  if chns ~= length(DigMin) then
    error("chns ~= length(DigMin)");
  end


// introduce some parameters that are refered to by id's
parameter1 = 12345;
vec = [ chns, smp_freq ];

   parlist = new_irparam_set();

   parlist = new_irparam_elemet_ivec(parlist, parameter1, 10); 
   parlist = new_irparam_elemet_ivec(parlist, vec, 11); // vector of integers (double vectors are similar, replace ivec with rvec)
   parlist = new_irparam_elemet_ivec(parlist, ascii(fname), 12);

   parlist = new_irparam_elemet_ivec(parlist, DigMax, 13); 
   parlist = new_irparam_elemet_ivec(parlist, DigMin, 14); 
   parlist = new_irparam_elemet_rvec(parlist, PhyMax, 15); 
   parlist = new_irparam_elemet_rvec(parlist, PhyMin, 16); 

   for i=1:Nin
     parlist = new_irparam_elemet_ivec(parlist, ascii( innames(i) ), 100+i-1);
   end

   p = combine_irparam(parlist);

// Set-up the block parameters and I/O ports

  Uipar = [ p.ipar ];
  Urpar = [ p.rpar ];
  btype = 15800 + 0; // Reference to the block's type (computational function). Use the same id you are giving via the "libdyn_compfnlist_add" C-function

  insizes=[ones( Nin,1 ) ];
  outsizes=[]; 
  dfeed=[];  // for each output 0 (no df) or 1 (a direct feedthrough to one of the inputs)
  intypes=  [ ones( Nin,1 )*ORTD.DATATYPE_FLOAT ];
  outtypes=[];

  blocktype = 1; // 1-BLOCKTYPE_DYNAMIC, 2-BLOCKTYPE_STATIC

  ///////////////
  [sim, blk] = libdyn_CreateBlockAutoConfig(sim, events, btype, blocktype, Uipar, Urpar, insizes, outsizes, intypes, outtypes, dfeed)
  /////////////
  
 [sim,blk] = libdyn_conn_equation(sim, blk, inlist );
//  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

// Interfacing functions are placed in this place







function [sim] = ld_Proc_shObj(sim, events, ObjectIdentifyer, Visibility, executable, OptionList) // PARSEDOCU_BLOCK
// 
// Set-up a Process with I/O redirection
//
// Note: only one and at least one parameter is required in OptionList
// 
// EXPERIMENTAL
// 

  // add a postfix that identifies the type of the shared object
  ObjectIdentifyer = ObjectIdentifyer + ".Process_ShObj";



   // pack all parameters into a structure "parlist"
   parlist = new_irparam_set();

   parlist = new_irparam_elemet_ivec(parlist, ascii(executable), 1); // id = 11; A string parameter

   NumberOfCmdOptions = length(OptionList);
   parlist = new_irparam_elemet_ivec(parlist, NumberOfCmdOptions, 10); // id = 10
   parlist = new_irparam_elemet_ivec(parlist, ascii(OptionList(1)), 11); // id = 11; A string parameter

   
   p = combine_irparam(parlist); // convert to two vectors of integers and floating point values respectively

// Set-up the block parameters. There are no I/O ports
  Uipar = [ p.ipar ];
  Urpar = [ p.rpar ];
  btype = 15300 + 1; // Reference to the block's type (computational function). Use the same id you are giving via the "libdyn_compfnlist_add" C-function


  [sim] =  libdyn_CreateSharedObjBlk(sim, btype, ObjectIdentifyer, Visibility, Uipar, Urpar); 
endfunction


function [sim, out, NumBytes] = ld_Process_Recv(sim, events, ObjectIdentifyer, ExpectedBytes, MaxOutsize) 
// 
// Process - receiver block
//
// out *, ORTD.DATATYPE_BINARY - output
// ExpectedBytes * ORTD.DATATYPE_INT32 - the number of bytes to read until an simulation step is triggered
// 
// This is a simulation-synchronising Block. Everytime ExpectedBytes are read from the process's stdout,
// the simulation that contains this blocks goes on for one step.
// 
// EXPERIMENTAL
// 

  printf("Synchronising simulation to Process-Receiver\n");

  // add a postfix that identifies the type of the shared object
  ObjectIdentifyer = ObjectIdentifyer + ".Process_ShObj";

  //
  outtype = ORTD.DATATYPE_BINARY;



   // pack all parameters into a structure "parlist"
   parlist = new_irparam_set();

   parlist = new_irparam_elemet_ivec(parlist, MaxOutsize, 10); // id = 10
   parlist = new_irparam_elemet_ivec(parlist, outtype, 11); // id = 11

   p = combine_irparam(parlist); // convert to two vectors of integers and floating point values respectively

// Set-up the block parameters and I/O ports
  Uipar = [ p.ipar ];
  Urpar = [ p.rpar ];
  btype = 15300 + 2; // Reference to the block's type (computational function). Use the same id you are giving via the "libdyn_compfnlist_add" C-function

  insizes=[1]; // Input port sizes
  outsizes=[MaxOutsize, 1]; // Output port sizes
  dfeed=[1];  // for each output 0 (no df) or 1 (a direct feedthrough to one of the inputs)
  intypes=[ORTD.DATATYPE_INT32]; // datatype for each input port
  outtypes=[outtype, ORTD.DATATYPE_INT32]; // datatype for each output port

  blocktype = 1; // 1-BLOCKTYPE_DYNAMIC (if block uses states), 2-BLOCKTYPE_STATIC (if there is only a static relationship between in- and output)

  // Create the block
  [sim, blk] = libdyn_CreateBlockAutoConfig(sim, events, btype, blocktype, Uipar, Urpar, insizes, outsizes, intypes, outtypes, dfeed, ObjectIdentifyer);
  
//   // connect the inputs
  [sim,blk] = libdyn_conn_equation(sim, blk, list(ExpectedBytes) ); // connect in1 to port 0 and in2 to port 1

   // connect the ouputs
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
  [sim,NumBytes] = libdyn_new_oport_hint(sim, blk, 1);   // 1th port
endfunction

function [sim] = ld_Process_Send(sim, events, SendSize, ObjectIdentifyer, in, insize) 
// 
// To process Send block
//
// in *, ORTD.DATATYPE_BINARY - input
// SendSize *. ORTD.DATATYPE_INT32 - Number of bytes to send
// 
// EXPERIMENTAL
// 

  intype = ORTD.DATATYPE_BINARY;

  // add a postfix that identifies the type of the shared object
  ObjectIdentifyer = ObjectIdentifyer + ".Process_ShObj";


   // pack all parameters into a structure "parlist"
   parlist = new_irparam_set();

   parlist = new_irparam_elemet_ivec(parlist, insize, 10); // id = 10
//    parlist = new_irparam_elemet_ivec(parlist, intype, 11); // id = 11

   p = combine_irparam(parlist); // convert to two vectors of integers and floating point values respectively

// Set-up the block parameters and I/O ports
  Uipar = [ p.ipar ];
  Urpar = [ p.rpar ];
  btype = 15300 + 3; // Reference to the block's type (computational function). Use the same id you are giving via the "libdyn_compfnlist_add" C-function

  insizes=[insize, 1]; // Input port sizes
  outsizes=[]; // Output port sizes
  dfeed=[1];  // for each output 0 (no df) or 1 (a direct feedthrough to one of the inputs)
  intypes=[intype,ORTD.DATATYPE_INT32]; // datatype for each input port
  outtypes=[]; // datatype for each output port

  blocktype = 1; // 1-BLOCKTYPE_DYNAMIC (if block uses states), 2-BLOCKTYPE_STATIC (if there is only a static relationship between in- and output)

  // Create the block
  [sim, blk] = libdyn_CreateBlockAutoConfig(sim, events, btype, blocktype, Uipar, Urpar, insizes, outsizes, intypes, outtypes, dfeed, ObjectIdentifyer);
  
  // connect the inputs
 [sim,blk] = libdyn_conn_equation(sim, blk, list(in, SendSize) ); // connect in1 to port 0 and in2 to port 1

//   // connect the ouputs
//  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction



















function [sim, out] = ld_startproc(sim, events, exepath, whentorun) // PARSEDOCU_BLOCK
// 
// %PURPOSE: Execute a sub process (EXPERIMENTAL)
//
// out * - output (unused)
// exepath - string: path to executable
// whentorun - 0 - Start process on blocks initialisation and stop on desruction; 
//             1 - Start process on activation in a statemachine and stop on reset
// 
// 

  printf("ld_startproc: starting %s\n", exepath);

 btype = 15300 + 0; // the same id you are giving via the "libdyn_compfnlist_add" C-function
 [sim,blk] = libdyn_new_block(sim, events, btype, ipar=[ 0, 0, 0, 0, whentorun,length(exepath),0, ascii(exepath)  ], rpar=[  ], ...
                  insizes=[], outsizes=[1], ...
                  intypes=[], outtypes=[ORTD.DATATYPE_FLOAT]  );

//  [sim,blk] = libdyn_conn_equation(sim, blk, list(in) );
 [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port


 [sim, out] = ld_gain(sim, events, out, 1); // FIXME: Otherwise the block above may not be initialised
endfunction

function [sim, out] = ld_startproc2(sim, events, exepath, chpwd, prio, whentorun) // PARSEDOCU_BLOCK
// 
// %PURPOSE: Execute a sub process with some more options (EXPERIMENTAL)
//
// out * - output
// exepath - string: path to executable
// chpwn - change current directory before running the process
// prio - Priority (set to zero for now)
// whentorun - 0 - Start process on blocks initialisation and stop on desruction; 
//             1 - Start process on activation in a statemachine and stop on reset
// 
// 

printf("ld_startproc: starting %s\n", exepath);

 btype = 15300 + 0; // the same id you are giving via the "libdyn_compfnlist_add" C-function
 [sim,blk] = libdyn_new_block(sim, events, btype, ipar=[ 0, 0, 0, prio, whentorun,length(exepath), length(chpwd), ascii(exepath), ascii(chpwd)  ], rpar=[  ], ...
                  insizes=[], outsizes=[1], ...
                  intypes=[], outtypes=[ORTD.DATATYPE_FLOAT]  );

//  [sim,blk] = libdyn_conn_equation(sim, blk, list(in) );
 [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port


 [sim, out] = ld_gain(sim, events, out, 1); // FIXME: Otherwise the block above may not be initialised
endfunction
// 
// 
// Interfacing functions are placed in this place
// 
// 
// This is a template from which scilab_loader.sce is automatically produced
// when running the module's makefile.
//
// The placeholder 39301 will be repalced when running the Makefile by the 
// contents of the variable blockid_start in the beginning of the Makefile
// 


// 
// ortd_checkpar types:
// 
//     'Signal' 
//     'SignalList' 
//     'SingleValue' 
//     'Vector'
//     'String'
// 
//  e.g.
// 
//   ortd_checkpar(sim, list('Signal', 'in', in) );
//   ortd_checkpar(sim, list('SingleValue', 'gain', gain) );
// 






// 
// How to use Shared Objects
// 
// 

function [sim] = ld_I2CDevice_shObj(sim, events, ObjectIdentifyer, Visibility, I2CDevicename, I2Caddr) // PARSEDOCU_BLOCK
// 
// %PURPOSE: I2C device connection
// 
// This function creates a shared object for I2C communications
// using the linux kernel's I2C-interface. (linux/i2c-dev.h, linux/i2c.h
// https://www.kernel.org/doc/Documentation/i2c/dev-interface )
// Hardware supporting this interface is e.g. available at the following systems
// 
// *) Beaglebone
// *) Rasperry Pi
// 
// An I2C device at adress I2Caddr ( numbering as used by i2cdetect; Please compare to other notations!)
// is opened via the I2C-bus given by I2CDevicename (e.g. "/dev/i2c-0", "/dev/i2c-1", ...)
// 
// Currently there are only functions for writing to the bus.
// 

  // add a postfix that identifies the type of the shared object
  ObjectIdentifyer = ObjectIdentifyer + ".I2CDevice_ShObj";


// introduce some parameters that are refered to by id's

   // pack all parameters into a structure "parlist"
   parlist = new_irparam_set();

   parlist = new_irparam_elemet_ivec(parlist, I2Caddr, 10); // id = 10
   parlist = new_irparam_elemet_ivec(parlist, ascii(I2CDevicename), 12); // id = 12; A string parameter

   p = combine_irparam(parlist); // convert to two vectors of integers and floating point values respectively

// Set-up the block parameters. There are no I/O ports
  Uipar = [ p.ipar ];
  Urpar = [ p.rpar ];
  btype = 39301 + 10; // Reference to the block's type (computational function). Use the same id you are giving via the "libdyn_compfnlist_add" C-function

  [sim] =  libdyn_CreateSharedObjBlk(sim, btype, ObjectIdentifyer, Visibility, Uipar, Urpar); 
endfunction


function [sim] = ld_I2CDevice_Write(sim, events, ObjectIdentifyer, Register, in) // PARSEDOCU_BLOCK
// 
// %PURPOSE: Write to a one byte register of an I2C-device
// 
// 
// 

  ortd_checkpar(sim, list('SingleValue', 'Register', Register) );
  ortd_checkpar(sim, list('Signal', 'in', in) );

  // add a postfix that identifies the type of the shared object
  ObjectIdentifyer = ObjectIdentifyer + ".I2CDevice_ShObj";

// introduce some parameters that are refered to by id's


   // pack all parameters into a structure "parlist"
   parlist = new_irparam_set();

   parlist = new_irparam_elemet_ivec(parlist, Register, 10); // id = 10

   p = combine_irparam(parlist); // convert to two vectors of integers and floating point values respectively

// Set-up the block parameters and I/O ports
  Uipar = [ p.ipar ];
  Urpar = [ p.rpar ];
  btype = 39301 + 11; // Reference to the block's type (computational function). Use the same id you are giving via the "libdyn_compfnlist_add" C-function

  insizes=[1]; // Input port sizes
  outsizes=[]; // Output port sizes
  dfeed=[1];  // for each output 0 (no df) or 1 (a direct feedthrough to one of the inputs)
  intypes=[ORTD.DATATYPE_INT32]; // datatype for each input port
  outtypes=[]; // datatype for each output port

  blocktype = 1; // 1-BLOCKTYPE_DYNAMIC (if block uses states), 2-BLOCKTYPE_STATIC (if there is only a static relationship between in- and output)

  // Create the block
  [sim, blk] = libdyn_CreateBlockAutoConfig(sim, events, btype, blocktype, Uipar, Urpar, insizes, outsizes, intypes, outtypes, dfeed, ObjectIdentifyer);
  
  // connect the inputs
 [sim,blk] = libdyn_conn_equation(sim, blk, list(in) ); // connect in1 to port 0 and in2 to port 1

  // connect the ouputs
//  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

function [sim] = ld_I2CDevice_BufferWrite(sim, events, ObjectIdentifyer, in, vecsize) // PARSEDOCU_BLOCK
// 
// %PURPOSE: Raw write of vecsize bytes to an I2C-bus.
// 
// The Values from multiple calls of this block are buffered untile they are
// send by ld_I2CDevice_Transmit.
// 
// Note: The maximal buffer size currently is 1000 bytes.
// 

  ortd_checkpar(sim, list('SingleValue', 'vecsize', vecsize) );
  ortd_checkpar(sim, list('Signal', 'in', in) );

  // add a postfix that identifies the type of the shared object
  ObjectIdentifyer = ObjectIdentifyer + ".I2CDevice_ShObj";

// introduce some parameters that are refered to by id's


   // pack all parameters into a structure "parlist"
   parlist = new_irparam_set();

//    parlist = new_irparam_elemet_ivec(parlist, Header, 10); // id = 10
//    parlist = new_irparam_elemet_ivec(parlist, Footer, 11); // id = 10

   p = combine_irparam(parlist); // convert to two vectors of integers and floating point values respectively

// Set-up the block parameters and I/O ports
  Uipar = [ p.ipar ];
  Urpar = [ p.rpar ];
  btype = 39301 + 12; // Reference to the block's type (computational function). Use the same id you are giving via the "libdyn_compfnlist_add" C-function

  insizes=[vecsize]; // Input port sizes
  outsizes=[]; // Output port sizes
  dfeed=[1];  // for each output 0 (no df) or 1 (a direct feedthrough to one of the inputs)
  intypes=[ORTD.DATATYPE_INT32]; // datatype for each input port
  outtypes=[]; // datatype for each output port

  blocktype = 1; // 1-BLOCKTYPE_DYNAMIC (if block uses states), 2-BLOCKTYPE_STATIC (if there is only a static relationship between in- and output)

  // Create the block
  [sim, blk] = libdyn_CreateBlockAutoConfig(sim, events, btype, blocktype, Uipar, Urpar, insizes, outsizes, intypes, outtypes, dfeed, ObjectIdentifyer);
  
  // connect the inputs
 [sim,blk] = libdyn_conn_equation(sim, blk, list(in) ); // connect in1 to port 0 and in2 to port 1

  // connect the ouputs
//  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

function [sim] = ld_I2CDevice_Transmit(sim, events, ObjectIdentifyer) // PARSEDOCU_BLOCK
// 
// %PURPOSE: Transmit / Flush write buffers and send the collected data to the device.
// 
// 

  // add a postfix that identifies the type of the shared object
  ObjectIdentifyer = ObjectIdentifyer + ".I2CDevice_ShObj";

// introduce some parameters that are refered to by id's


   // pack all parameters into a structure "parlist"
   parlist = new_irparam_set();

//    parlist = new_irparam_elemet_ivec(parlist, Header, 10); // id = 10
//    parlist = new_irparam_elemet_ivec(parlist, Footer, 11); // id = 10

   p = combine_irparam(parlist); // convert to two vectors of integers and floating point values respectively

// Set-up the block parameters and I/O ports
  Uipar = [ p.ipar ];
  Urpar = [ p.rpar ];
  btype = 39301 + 13; // Reference to the block's type (computational function). Use the same id you are giving via the "libdyn_compfnlist_add" C-function

  insizes=[1]; // Input port sizes
  outsizes=[]; // Output port sizes
  dfeed=[1];  // for each output 0 (no df) or 1 (a direct feedthrough to one of the inputs)
  intypes=[ORTD.DATATYPE_FLOAT]; // datatype for each input port
  outtypes=[]; // datatype for each output port

  blocktype = 1; // 1-BLOCKTYPE_DYNAMIC (if block uses states), 2-BLOCKTYPE_STATIC (if there is only a static relationship between in- and output)

  // Create the block
  [sim, blk] = libdyn_CreateBlockAutoConfig(sim, events, btype, blocktype, Uipar, Urpar, insizes, outsizes, intypes, outtypes, dfeed, ObjectIdentifyer);
  
  // connect the inputs
  [sim, dummy] = ld_const(sim, 0, 0); // dummy input FIXM. There is a function for this
 [sim,blk] = libdyn_conn_equation(sim, blk, list(dummy) ); // connect in1 to port 0 and in2 to port 1

  // connect the ouputs
//  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction



function [sim, out] = ld_I2CDevice_Read(sim, events, ObjectIdentifyer, Register) // PARSEDOCU_BLOCK
// 
// %PURPOSE: read one byte from a register of an I2C-device
// 
// 
// 

  ortd_checkpar(sim, list('SingleValue', 'Register', Register) );
//   ortd_checkpar(sim, list('Signal', 'in', in) );

  // add a postfix that identifies the type of the shared object
  ObjectIdentifyer = ObjectIdentifyer + ".I2CDevice_ShObj";

// introduce some parameters that are refered to by id's


   // pack all parameters into a structure "parlist"
   parlist = new_irparam_set();

   parlist = new_irparam_elemet_ivec(parlist, Register, 10); // id = 10

   p = combine_irparam(parlist); // convert to two vectors of integers and floating point values respectively

// Set-up the block parameters and I/O ports
  Uipar = [ p.ipar ];
  Urpar = [ p.rpar ];
  btype = 39301 + 14; // Reference to the block's type (computational function). Use the same id you are giving via the "libdyn_compfnlist_add" C-function

  insizes=[]; // Input port sizes
  outsizes=[1]; // Output port sizes
  dfeed=[1];  // for each output 0 (no df) or 1 (a direct feedthrough to one of the inputs)
  intypes=[]; // datatype for each input port
  outtypes=[ORTD.DATATYPE_INT32]; // datatype for each output port

  blocktype = 1; // 1-BLOCKTYPE_DYNAMIC (if block uses states), 2-BLOCKTYPE_STATIC (if there is only a static relationship between in- and output)

  // Create the block
  [sim, blk] = libdyn_CreateBlockAutoConfig(sim, events, btype, blocktype, Uipar, Urpar, insizes, outsizes, intypes, outtypes, dfeed, ObjectIdentifyer);
  
  // connect the inputs
 [sim,blk] = libdyn_conn_equation(sim, blk, list() ); // connect in1 to port 0 and in2 to port 1

  // connect the ouputs
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction




function [x,y,typ] = hart_powermate(job,arg1,arg2)

  x=[];y=[];typ=[];
  select job
  case 'plot' then
    exprs=arg1.graphics.exprs;
    standard_draw(arg1)
  case 'getinputs' then
    [x,y,typ]=standard_inputs(arg1)
  case 'getoutputs' then
    [x,y,typ]=standard_outputs(arg1)
  case 'getorigin' then
    [x,y]=standard_origin(arg1)
  case 'set' then
    x=arg1
    model=arg1.model;graphics=arg1.graphics;
    exprs=graphics.exprs;
    while %t do
  try
  getversion('scilab');
      [ok,min_value,max_value,rounds,start,btn_mode,exprs]=..
      scicos_getvalue('Set POWERMATE block parameters',..
         ['Min. Value:';
         'Max Value:';
         'Tours between min and  max:';
         'Start Value:';
          '2nd Output - Press(0)/Toggle(1):'],..
      list('vec',-1,'vec',-1,'vec',-1,'vec',-1,'vec',-1),exprs)
catch
      [ok,min_value,max_value,rounds,start,btn_mode,exprs]=..
      getvalue('Set POWERMATE block parameters',..
         ['Min. Value:';
         'Max Value:';
         'Tours between min and  max:';
         'Start Value:';
          '2nd Output - Press(0)/Toggle(1):'],..
      list('vec',-1,'vec',-1,'vec',-1,'vec',-1,'vec',-1),exprs)
end;
     if ~ok then break,end
      in=[model.in]
      out=[model.out]
      evtin=[1]
      evtout=[]
      [model,graphics,ok]=check_io(model,graphics,in,out,evtin,evtout);
      if ok then
        graphics.exprs=exprs;
        model.ipar=[];
        model.rpar=[min_value;
max_value;
rounds;
start;
btn_mode
];
   model.dstate=[1];
        x.graphics=graphics;x.model=model
        break
      end
    end
  case 'define' then
     min_value=0;
     max_value=1;
     rounds=5;
     start=0;
     btn_mode=1;
   model=scicos_model()
   model.sim=list('rt_powermate',4)
   model.in=[]
   model.out=[ones(2,1)]
   model.evtin=[1]
   model.evtout=[]
   model.ipar=[];
   model.rpar=[min_value;
max_value;
rounds;
start;
btn_mode
];
 model.dstate=[1];
 model.blocktype='d';
 model.dep_ut=[%t %f];
    exprs=[sci2exp(min_value);sci2exp(max_value);sci2exp(rounds);sci2exp(start);sci2exp(btn_mode)]
    gr_i=['xstringb(orig(1),orig(2),[''POWERMATE'' ],sz(1),sz(2),''fill'');'];
    x=standard_define([3 2],model,exprs,gr_i)
case 'readout' then
      BLOCK.version=020;
      BLOCK.name='hart_powermate';
      BLOCK.comp_name='rt_powermate';
      BLOCK.desr_short='Set POWERMATE block parameters';
      BLOCK.dep_u=%t;
      BLOCK.dep_t=%f;
      BLOCK.blocktype='d';
      BLOCK.dstate='1';
      BLOCK.IOmatrix=%f;
      BLOCK.inset=%f;
      BLOCK.in='';
      BLOCK.outset=%f;
      BLOCK.out='ones(2,1)';
      BLOCK.evtin='1';
      BLOCK.evtout='';
      BLOCK.size='3 2';
      BLOCK.completelabel=%f;
      BLOCK.label=[39,39,80,79,87,69,82,77,65,84,69,39,39,10];
      BLOCK.ipar=[];
      BLOCK.rpar=[109,105,110,95,118,97,108,117,101,59,10,109,97,120,95,118,97,108,117,101,59,10,114,111,..
         117,110,100,115,59,10,115,116,97,114,116,59,10,98,116,110,95,109,111,100,101,10];
      BLOCK.opar=[];
      BLOCK.parameter=list();
      BLOCK.parameter($+1)=[];
      BLOCK.parameter(1).name='min_value';
      BLOCK.parameter(1).text='Min. Value:';
      BLOCK.parameter(1).type='vec';
      BLOCK.parameter(1).size='-1';
      BLOCK.parameter(1).init='0';
      BLOCK.parameter(1).visible_plot=%f;
      BLOCK.parameter($+1)=[];
      BLOCK.parameter(2).name='max_value';
      BLOCK.parameter(2).text='Max Value:';
      BLOCK.parameter(2).type='vec';
      BLOCK.parameter(2).size='-1';
      BLOCK.parameter(2).init='1';
      BLOCK.parameter(2).visible_plot=%f;
      BLOCK.parameter($+1)=[];
      BLOCK.parameter(3).name='rounds';
      BLOCK.parameter(3).text='Tours between min and  max:';
      BLOCK.parameter(3).type='vec';
      BLOCK.parameter(3).size='-1';
      BLOCK.parameter(3).init='5';
      BLOCK.parameter(3).visible_plot=%f;
      BLOCK.parameter($+1)=[];
      BLOCK.parameter(4).name='start';
      BLOCK.parameter(4).text='Start Value:';
      BLOCK.parameter(4).type='vec';
      BLOCK.parameter(4).size='-1';
      BLOCK.parameter(4).init='0';
      BLOCK.parameter(4).visible_plot=%f;
      BLOCK.parameter($+1)=[];
      BLOCK.parameter(5).name='btn_mode';
      BLOCK.parameter(5).text='2nd Output - Press(0)/Toggle(1):';
      BLOCK.parameter(5).type='vec';
      BLOCK.parameter(5).size='-1';
      BLOCK.parameter(5).init='1';
      BLOCK.parameter(5).visible_plot=%f;
      x=BLOCK;
  end
endfunction
function [x,y,typ] = hart_sciencemode(job,arg1,arg2)
//Stimulator Interface (RehaStim - Hasomed GmbH)
//
// Block Screenshot
//
// Description
//
//     Current 
// 
//     This input should be an array of values representing the currents desired on the channels specified in the block parameters.  Thus if the specified channels are [1 5 6] then there should be 3 inputs into this port, the first being for the stimulation channel 1 the second for stimulation channel 5 and the last for stimulation channel 6.
//     
//         Pulsewidth
// 
//     Similar to the current this input should also be an array of values, representing the pulsewidths desired on the channels specified in the block parameters. 
//     
//         Mode 
// 
//   Similar to the current this input should be an array of values representing the mode desired for each channels specified in the block parameters. The value of the mode should be either 0, 1 or 2:
//       0 is for singlet stimulation pulses.
//        1 is for doublet stimulation pulses.
//       2 is for tripplet stimulation pulses.
//
//
// Dialog box
//
// Serial Port:  The serial port connected to the stimulator.
// Channels to be Stimulated:  0 An array of channel numbers to be used (e.g. [1 2 5]).
// Main Time in ms (only in steps of 0.5ms):       The stimulation period (set this to 0 for external triggering).  For example, if a stimulation frequency of 50Hz is desired then this value should be set to 20ms.  This parameter can be set to 0ms to activate the external triggering of the stimulation pulse (or pulse group).  In this case the channels listed in the "Channels to be Stimulated" parameter will all be triggered each time this block is evaluated.  There is a minimum limit on this value defined by the Group Time and by the maximum mode input.
//Group Time in ms (only in steps of 0.5ms):   This parameter is the time between pulses in a doublet or triplet group.
// Low Freq Ch:      It is possible to set some channels to use a much lower frequency than the main frequency dictated by the Main Time parameter.  This is a useful feature when applying a mixed reflex and muscle stimulation pattern. The values listed in this parameter must also be listed in the "Channels to be Stimulated" parameter
// Low Freq Fc:         Rather than send a pulse (or pulse group) every time a channel is triggered, a pulse could be sent every nth time it is triggered.  Thus n is the Low Frequency Factor.
//
// Default Properties
//
// always active: yes
// direct-feedthrough: no
// zero-crossing: no
// mode: no
// regular inputs:  port 1 : size [1,1] / type 1
// regular outputs: port 1 : size [1,1] / type 1
// number/sizes of activation inputs: 1
// number/sizes of activation outputs: 0
// continuous-time state: no
// discrete-time state: yes
// object discrete-time state: no
// name of computational function: rt_par2ser
//  
//
// Interfacing Function
// hart_sciencemode.sci
// Computational Function
// hart_sciencemode.cpp
// Authors
// Holger Nahrstaedt
//

  x=[];y=[];typ=[];
  select job
  case 'plot' then
    exprs=arg1.graphics.exprs;
    name=exprs(1);
    standard_draw(arg1)
  case 'getinputs' then
    [x,y,typ]=standard_inputs(arg1)
  case 'getoutputs' then
    [x,y,typ]=standard_outputs(arg1)
  case 'getorigin' then
    [x,y]=standard_origin(arg1)
  case 'set' then
    x=arg1
    model=arg1.model;graphics=arg1.graphics;
    exprs=graphics.exprs;
    while %t do
  try
  getversion('scilab');
      [ok,name,channels,main_time,group_time,channels_LF,lowFreqFc,exprs]=..
      scicos_getvalue('Set RTAI SCIENCEMODE block parameters',..
         ['Device (add ending): /dev/Channels:';
         'Channels to be Stimulated (e.g. [1 2 5])';
         'Main Time in ms: The stimulation period (0 for external triggering)';
         'Group Time in ms: The time between pulses in a group (doublet or triplet)';
         'Low Freq Ch:   A sub-set of the channels selected for low frequency';
          'Low Freq Fc:   Stimulation is only every n times (n=factor)'],..
      list('str',1,'vec',-1,'vec',-1,'vec',-1,'vec',-1,'vec',-1),exprs)
catch
      [ok,name,channels,main_time,group_time,channels_LF,lowFreqFc,exprs]=..
      getvalue('Set RTAI SCIENCEMODE block parameters',..
         ['Device (add ending): /dev/Channels:';
         'Channels to be Stimulated (e.g. [1 2 5])';
         'Main Time in ms: The stimulation period (0 for external triggering)';
         'Group Time in ms: The time between pulses in a group (doublet or triplet)';
         'Low Freq Ch:   A sub-set of the channels selected for low frequency';
          'Low Freq Fc:   Stimulation is only every n times (n=factor)'],..
      list('str',1,'vec',-1,'vec',-1,'vec',-1,'vec',-1,'vec',-1),exprs)
end;
     if ~ok then break,end
      in=[ones(3,1)*length(channels)]
      out=[model.out]
      evtin=[1]
      evtout=[]
      [model,graphics,ok]=check_io(model,graphics,in,out,evtin,evtout);
      if ok then
        graphics.exprs=exprs;
        model.ipar=[length(channels);
channels';
length(channels_LF);
channels_LF';
lowFreqFc;
length(name);
ascii(name)'
];
        model.rpar=[main_time;
group_time
];
   model.dstate=[];
        x.graphics=graphics;x.model=model
        break
      end
    end
  case 'define' then
     name='stimulator';
     channels=[1 2];
     main_time=0;
     group_time=10;
     channels_LF=[];
     lowFreqFc=0;
   model=scicos_model()
   model.sim=list('rt_sciencemode',4)
   model.in=[ones(3,1)*length(channels)]
   model.out=[]
   model.evtin=[1]
   model.evtout=[]
   model.ipar=[length(channels);
channels';
length(channels_LF);
channels_LF';
lowFreqFc;
length(name);
ascii(name)'
];
   model.rpar=[main_time;
group_time
];
 model.dstate=[];
 model.blocktype='d';
 model.dep_ut=[%f %f];
    exprs=[name;sci2exp(channels);sci2exp(main_time);sci2exp(group_time);sci2exp(channels_LF);sci2exp(lowFreqFc)]
    gr_i=['xstringb(orig(1)+sz(1)/2,orig(2),[''Sciencemode'';name],sz(1)/2,sz(2),''fill'');',           'xstringb(orig(1)+sz(1)/6,orig(2)+sz(2)/4,[''pulse width'';],sz(1)/10,sz(2),''fill'');',         'xstringb(orig(1)+sz(1)/6,orig(2),[''current''],sz(1)/10,sz(2),''fill'');',         'xstringb(orig(1)+sz(1)/6,orig(2)-sz(2)/4,[''mode''],sz(1)/10,sz(2),''fill'');' ];
    x=standard_define([5 4],model,exprs,gr_i)
case 'readout' then
      BLOCK.version=020;
      BLOCK.name='hart_sciencemode';
      BLOCK.comp_name='rt_sciencemode';
      BLOCK.desr_short='Set RTAI SCIENCEMODE block parameters';
      BLOCK.dep_u=%f;
      BLOCK.dep_t=%f;
      BLOCK.blocktype='d';
      BLOCK.dstate='';
      BLOCK.IOmatrix=%f;
      BLOCK.inset=%t;
      BLOCK.in='ones(3,1)*length(channels)';
      BLOCK.outset=%f;
      BLOCK.out='';
      BLOCK.evtin='1';
      BLOCK.evtout='';
      BLOCK.size='5 4';
      BLOCK.completelabel=%t;
      BLOCK.label=[39,120,115,116,114,105,110,103,98,40,111,114,105,103,40,49,41,43,115,122,40,49,41,47,50,..
         44,111,114,105,103,40,50,41,44,91,39,39,83,99,105,101,110,99,101,109,111,100,101,39,39,..
         59,110,97,109,101,93,44,115,122,40,49,41,47,50,44,115,122,40,50,41,44,39,39,102,105,108,..
         108,39,39,41,59,39,44,10,32,32,32,32,32,32,32,32,32,32,39,120,115,116,114,105,110,103,98,..
         40,111,114,105,103,40,49,41,43,115,122,40,49,41,47,54,44,111,114,105,103,40,50,41,43,115,..
         122,40,50,41,47,52,44,91,39,39,112,117,108,115,101,32,119,105,100,116,104,39,39,59,93,44,..
         115,122,40,49,41,47,49,48,44,115,122,40,50,41,44,39,39,102,105,108,108,39,39,41,59,39,44,..
         10,32,32,32,32,32,32,32,32,39,120,115,116,114,105,110,103,98,40,111,114,105,103,40,49,41,..
         43,115,122,40,49,41,47,54,44,111,114,105,103,40,50,41,44,91,39,39,99,117,114,114,101,110,..
         116,39,39,93,44,115,122,40,49,41,47,49,48,44,115,122,40,50,41,44,39,39,102,105,108,108,..
         39,39,41,59,39,44,10,32,32,32,32,32,32,32,32,39,120,115,116,114,105,110,103,98,40,111,..
         114,105,103,40,49,41,43,115,122,40,49,41,47,54,44,111,114,105,103,40,50,41,45,115,122,40,..
         50,41,47,52,44,91,39,39,109,111,100,101,39,39,93,44,115,122,40,49,41,47,49,48,44,115,122,..
         40,50,41,44,39,39,102,105,108,108,39,39,41,59,39,10];
      BLOCK.ipar=[108,101,110,103,116,104,40,99,104,97,110,110,101,108,115,41,59,10,99,104,97,110,110,101,..
         108,115,39,59,10,108,101,110,103,116,104,40,99,104,97,110,110,101,108,115,95,76,70,41,59,..
         10,99,104,97,110,110,101,108,115,95,76,70,39,59,10,108,111,119,70,114,101,113,70,99,59,..
         10,108,101,110,103,116,104,40,110,97,109,101,41,59,10,97,115,99,105,105,40,110,97,109,..
         101,41,39,10];
      BLOCK.rpar=[109,97,105,110,95,116,105,109,101,59,10,103,114,111,117,112,95,116,105,109,101,10];
      BLOCK.opar=[];
      BLOCK.parameter=list();
      BLOCK.parameter($+1)=[];
      BLOCK.parameter(1).name='name';
      BLOCK.parameter(1).text='Device (add ending): /dev/Channels:';
      BLOCK.parameter(1).type='str';
      BLOCK.parameter(1).size='1';
      BLOCK.parameter(1).init='stimulator';
      BLOCK.parameter(1).visible_plot=%t;
      BLOCK.parameter($+1)=[];
      BLOCK.parameter(2).name='channels';
      BLOCK.parameter(2).text='Channels to be Stimulated (e.g. [1 2 5])';
      BLOCK.parameter(2).type='vec';
      BLOCK.parameter(2).size='-1';
      BLOCK.parameter(2).init='[1 2]';
      BLOCK.parameter(2).visible_plot=%f;
      BLOCK.parameter($+1)=[];
      BLOCK.parameter(3).name='main_time';
      BLOCK.parameter(3).text='Main Time in ms: The stimulation period (0 for external triggering)';
      BLOCK.parameter(3).type='vec';
      BLOCK.parameter(3).size='-1';
      BLOCK.parameter(3).init='0';
      BLOCK.parameter(3).visible_plot=%f;
      BLOCK.parameter($+1)=[];
      BLOCK.parameter(4).name='group_time';
      BLOCK.parameter(4).text='Group Time in ms: The time between pulses in a group (doublet or triplet)';
      BLOCK.parameter(4).type='vec';
      BLOCK.parameter(4).size='-1';
      BLOCK.parameter(4).init='10';
      BLOCK.parameter(4).visible_plot=%f;
      BLOCK.parameter($+1)=[];
      BLOCK.parameter(5).name='channels_LF';
      BLOCK.parameter(5).text='Low Freq Ch:   A sub-set of the channels selected for low frequency';
      BLOCK.parameter(5).type='vec';
      BLOCK.parameter(5).size='-1';
      BLOCK.parameter(5).init='';
      BLOCK.parameter(5).visible_plot=%f;
      BLOCK.parameter($+1)=[];
      BLOCK.parameter(6).name='lowFreqFc';
      BLOCK.parameter(6).text='Low Freq Fc:   Stimulation is only every n times (n=factor)';
      BLOCK.parameter(6).type='vec';
      BLOCK.parameter(6).size='-1';
      BLOCK.parameter(6).init='0';
      BLOCK.parameter(6).visible_plot=%f;
      x=BLOCK;
  end
endfunction
function [x,y,typ] = hart_sciencemode_rt(job,arg1,arg2)
//Stimulator Interface (RehaStim - Hasomed GmbH)
//
// Block Screenshot
//
// Description
//
//     Current 
// 
//     This input should be an array of values representing the currents desired on the channels specified in the block parameters.  Thus if the specified channels are [1 5 6] then there should be 3 inputs into this port, the first being for the stimulation channel 1 the second for stimulation channel 5 and the last for stimulation channel 6.
//     
//         Pulsewidth
// 
//     Similar to the current this input should also be an array of values, representing the pulsewidths desired on the channels specified in the block parameters. 
//     
//         Mode 
// 
//   Similar to the current this input should be an array of values representing the mode desired for each channels specified in the block parameters. The value of the mode should be either 0, 1 or 2:
//       0 is for singlet stimulation pulses.
//        1 is for doublet stimulation pulses.
//       2 is for tripplet stimulation pulses.
//
//
// Dialog box
//
// Serial Port:  The serial port connected to the stimulator.
// Channels to be Stimulated:  0 An array of channel numbers to be used (e.g. [1 2 5]).
// Main Time in ms (only in steps of 0.5ms):       The stimulation period (set this to 0 for external triggering).  For example, if a stimulation frequency of 50Hz is desired then this value should be set to 20ms.  This parameter can be set to 0ms to activate the external triggering of the stimulation pulse (or pulse group).  In this case the channels listed in the "Channels to be Stimulated" parameter will all be triggered each time this block is evaluated.  There is a minimum limit on this value defined by the Group Time and by the maximum mode input.
//Group Time in ms (only in steps of 0.5ms):   This parameter is the time between pulses in a doublet or triplet group.
// Low Freq Ch:      It is possible to set some channels to use a much lower frequency than the main frequency dictated by the Main Time parameter.  This is a useful feature when applying a mixed reflex and muscle stimulation pattern. The values listed in this parameter must also be listed in the "Channels to be Stimulated" parameter
// Low Freq Fc:         Rather than send a pulse (or pulse group) every time a channel is triggered, a pulse could be sent every nth time it is triggered.  Thus n is the Low Frequency Factor.
//
// Default Properties
//
// always active: yes
// direct-feedthrough: no
// zero-crossing: no
// mode: no
// regular inputs:  port 1 : size [1,1] / type 1
// regular outputs: port 1 : size [1,1] / type 1
// number/sizes of activation inputs: 1
// number/sizes of activation outputs: 0
// continuous-time state: no
// discrete-time state: yes
// object discrete-time state: no
// name of computational function: rt_par2ser
//  
//
// Interfacing Function
// hart_sciencemode_rt.sci
// Computational Function
// hart_sciencemode_supp.cpp
// Authors
// Holger Nahrstaedt
//

  x=[];y=[];typ=[];
  select job
  case 'plot' then
    exprs=arg1.graphics.exprs;
    name=exprs(1);
    standard_draw(arg1)
  case 'getinputs' then
    [x,y,typ]=standard_inputs(arg1)
  case 'getoutputs' then
    [x,y,typ]=standard_outputs(arg1)
  case 'getorigin' then
    [x,y]=standard_origin(arg1)
  case 'set' then
    x=arg1
    model=arg1.model;graphics=arg1.graphics;
    exprs=graphics.exprs;
    while %t do
  try
  getversion('scilab');
      [ok,name,channels,main_time,group_time,channels_LF,lowFreqFc,exprs]=..
      scicos_getvalue('Set RTAI SCIENCEMODE block parameters',..
         ['Device (add ending): /dev/Channels:';
         'Channels to be Stimulated (e.g. [1 2 5])';
         'Main Time in ms: The stimulation period (0 for external triggering)';
         'Group Time in ms: The time between pulses in a group (doublet or triplet)';
         'Low Freq Ch:   A sub-set of the channels selected for low frequency';
          'Low Freq Fc:   Stimulation is only every n times (n=factor)'],..
      list('str',1,'vec',-1,'vec',-1,'vec',-1,'vec',-1,'vec',-1),exprs)
catch
      [ok,name,channels,main_time,group_time,channels_LF,lowFreqFc,exprs]=..
      getvalue('Set RTAI SCIENCEMODE block parameters',..
         ['Device (add ending): /dev/Channels:';
         'Channels to be Stimulated (e.g. [1 2 5])';
         'Main Time in ms: The stimulation period (0 for external triggering)';
         'Group Time in ms: The time between pulses in a group (doublet or triplet)';
         'Low Freq Ch:   A sub-set of the channels selected for low frequency';
          'Low Freq Fc:   Stimulation is only every n times (n=factor)'],..
      list('str',1,'vec',-1,'vec',-1,'vec',-1,'vec',-1,'vec',-1),exprs)
end;
     if ~ok then break,end
      in=[ones(3,1)*length(channels)]
      out=[model.out]
      evtin=[1]
      evtout=[]
      [model,graphics,ok]=check_io(model,graphics,in,out,evtin,evtout);
      if ok then
        graphics.exprs=exprs;
        model.ipar=[length(channels);
channels';
length(channels_LF);
channels_LF';
lowFreqFc;
length(name);
ascii(name)'
];
        model.rpar=[main_time;
group_time
];
   model.dstate=[0];
        x.graphics=graphics;x.model=model
        break
      end
    end
  case 'define' then
     name='stimulator';
     channels=[1 2];
     main_time=0;
     group_time=10;
     channels_LF=[];
     lowFreqFc=0;
   model=scicos_model()
   model.sim=list('rt_sciencemode_sup',4)
   model.in=[ones(3,1)*length(channels)]
   model.out=[1]
   model.evtin=[1]
   model.evtout=[]
   model.ipar=[length(channels);
channels';
length(channels_LF);
channels_LF';
lowFreqFc;
length(name);
ascii(name)'
];
   model.rpar=[main_time;
group_time
];
 model.dstate=[0];
 model.blocktype='d';
 model.dep_ut=[%f %f];
    exprs=[name;sci2exp(channels);sci2exp(main_time);sci2exp(group_time);sci2exp(channels_LF);sci2exp(lowFreqFc)]
    gr_i=['xstringb(orig(1)+sz(1)/2,orig(2),[''Sciencemode RT'';name],sz(1)/2,sz(2),''fill'');',           'xstringb(orig(1)+sz(1)/6,orig(2)+sz(2)/4,[''pulse width'';],sz(1)/10,sz(2),''fill'');',         'xstringb(orig(1)+sz(1)/6,orig(2),[''current''],sz(1)/10,sz(2),''fill'');',         'xstringb(orig(1)+sz(1)/6,orig(2)-sz(2)/4,[''mode''],sz(1)/10,sz(2),''fill'');'  ];
    x=standard_define([5 4],model,exprs,gr_i)
case 'readout' then
      BLOCK.version=020;
      BLOCK.name='hart_sciencemode_rt';
      BLOCK.comp_name='rt_sciencemode_sup';
      BLOCK.desr_short='Set RTAI SCIENCEMODE block parameters';
      BLOCK.dep_u=%f;
      BLOCK.dep_t=%f;
      BLOCK.blocktype='d';
      BLOCK.dstate='0';
      BLOCK.IOmatrix=%f;
      BLOCK.inset=%t;
      BLOCK.in='ones(3,1)*length(channels)';
      BLOCK.outset=%f;
      BLOCK.out='1';
      BLOCK.evtin='1';
      BLOCK.evtout='';
      BLOCK.size='5 4';
      BLOCK.completelabel=%t;
      BLOCK.label=[39,120,115,116,114,105,110,103,98,40,111,114,105,103,40,49,41,43,115,122,40,49,41,47,50,..
         44,111,114,105,103,40,50,41,44,91,39,39,83,99,105,101,110,99,101,109,111,100,101,32,82,..
         84,39,39,59,110,97,109,101,93,44,115,122,40,49,41,47,50,44,115,122,40,50,41,44,39,39,102,..
         105,108,108,39,39,41,59,39,44,10,32,32,32,32,32,32,32,32,32,32,39,120,115,116,114,105,..
         110,103,98,40,111,114,105,103,40,49,41,43,115,122,40,49,41,47,54,44,111,114,105,103,40,..
         50,41,43,115,122,40,50,41,47,52,44,91,39,39,112,117,108,115,101,32,119,105,100,116,104,..
         39,39,59,93,44,115,122,40,49,41,47,49,48,44,115,122,40,50,41,44,39,39,102,105,108,108,39,..
         39,41,59,39,44,10,32,32,32,32,32,32,32,32,39,120,115,116,114,105,110,103,98,40,111,114,..
         105,103,40,49,41,43,115,122,40,49,41,47,54,44,111,114,105,103,40,50,41,44,91,39,39,99,..
         117,114,114,101,110,116,39,39,93,44,115,122,40,49,41,47,49,48,44,115,122,40,50,41,44,39,..
         39,102,105,108,108,39,39,41,59,39,44,10,32,32,32,32,32,32,32,32,39,120,115,116,114,105,..
         110,103,98,40,111,114,105,103,40,49,41,43,115,122,40,49,41,47,54,44,111,114,105,103,40,..
         50,41,45,115,122,40,50,41,47,52,44,91,39,39,109,111,100,101,39,39,93,44,115,122,40,49,41,..
         47,49,48,44,115,122,40,50,41,44,39,39,102,105,108,108,39,39,41,59,39,10,10];
      BLOCK.ipar=[108,101,110,103,116,104,40,99,104,97,110,110,101,108,115,41,59,10,99,104,97,110,110,101,..
         108,115,39,59,10,108,101,110,103,116,104,40,99,104,97,110,110,101,108,115,95,76,70,41,59,..
         10,99,104,97,110,110,101,108,115,95,76,70,39,59,10,108,111,119,70,114,101,113,70,99,59,..
         10,108,101,110,103,116,104,40,110,97,109,101,41,59,10,97,115,99,105,105,40,110,97,109,..
         101,41,39,10];
      BLOCK.rpar=[109,97,105,110,95,116,105,109,101,59,10,103,114,111,117,112,95,116,105,109,101,10];
      BLOCK.opar=[];
      BLOCK.parameter=list();
      BLOCK.parameter($+1)=[];
      BLOCK.parameter(1).name='name';
      BLOCK.parameter(1).text='Device (add ending): /dev/Channels:';
      BLOCK.parameter(1).type='str';
      BLOCK.parameter(1).size='1';
      BLOCK.parameter(1).init='stimulator';
      BLOCK.parameter(1).visible_plot=%t;
      BLOCK.parameter($+1)=[];
      BLOCK.parameter(2).name='channels';
      BLOCK.parameter(2).text='Channels to be Stimulated (e.g. [1 2 5])';
      BLOCK.parameter(2).type='vec';
      BLOCK.parameter(2).size='-1';
      BLOCK.parameter(2).init='[1 2]';
      BLOCK.parameter(2).visible_plot=%f;
      BLOCK.parameter($+1)=[];
      BLOCK.parameter(3).name='main_time';
      BLOCK.parameter(3).text='Main Time in ms: The stimulation period (0 for external triggering)';
      BLOCK.parameter(3).type='vec';
      BLOCK.parameter(3).size='-1';
      BLOCK.parameter(3).init='0';
      BLOCK.parameter(3).visible_plot=%f;
      BLOCK.parameter($+1)=[];
      BLOCK.parameter(4).name='group_time';
      BLOCK.parameter(4).text='Group Time in ms: The time between pulses in a group (doublet or triplet)';
      BLOCK.parameter(4).type='vec';
      BLOCK.parameter(4).size='-1';
      BLOCK.parameter(4).init='10';
      BLOCK.parameter(4).visible_plot=%f;
      BLOCK.parameter($+1)=[];
      BLOCK.parameter(5).name='channels_LF';
      BLOCK.parameter(5).text='Low Freq Ch:   A sub-set of the channels selected for low frequency';
      BLOCK.parameter(5).type='vec';
      BLOCK.parameter(5).size='-1';
      BLOCK.parameter(5).init='';
      BLOCK.parameter(5).visible_plot=%f;
      BLOCK.parameter($+1)=[];
      BLOCK.parameter(6).name='lowFreqFc';
      BLOCK.parameter(6).text='Low Freq Fc:   Stimulation is only every n times (n=factor)';
      BLOCK.parameter(6).type='vec';
      BLOCK.parameter(6).size='-1';
      BLOCK.parameter(6).init='0';
      BLOCK.parameter(6).visible_plot=%f;
      x=BLOCK;
  end
endfunction
function [x,y,typ] = imu_algorithm(job,arg1,arg2)

  x=[];y=[];typ=[];
  select job
  case 'plot' then
    exprs=arg1.graphics.exprs;
    standard_draw(arg1)
  case 'getinputs' then
    [x,y,typ]=standard_inputs(arg1)
  case 'getoutputs' then
    [x,y,typ]=standard_outputs(arg1)
  case 'getorigin' then
    [x,y]=standard_origin(arg1)
  case 'set' then
    x=arg1
    model=arg1.model;graphics=arg1.graphics;
    exprs=graphics.exprs;
    while %t do
      [ok,Kp_rp,Ki_rp,Kp_y,Ki_y,exprs]=..
      getvalue('imu algorithm',..
         ['Kp roll pitch:';
         'Kp roll pitch:';
         'Kp Yaw';
          'Ki Yaw'],..
      list('vec',1,'vec',1,'vec',1,'vec',1),exprs)
     if ~ok then break,end
      in=[model.in]
      out=[model.out]
      evtin=[1]
      evtout=[]
      [model,graphics,ok]=check_io(model,graphics,in,out,evtin,evtout);
      if ok then
        graphics.exprs=exprs;
        model.ipar=[1
];
        model.rpar=[Kp_rp;
Ki_rp;
Kp_y;
Ki_y
];
	  model.dstate=[];
        x.graphics=graphics;x.model=model
        break
      end
    end
  case 'define' then
     Kp_rp=0.6;
     Ki_rp=0.00002;
     Kp_y=1.2;
     Ki_y=0.00002;
   model=scicos_model()
   model.sim=list('rt_imu_algorithm',4)
   model.in=[9]
   model.out=[[9;3;3]]
   model.evtin=[1]
   model.evtout=[]
   model.ipar=[1
];
   model.rpar=[Kp_rp;
Ki_rp;
Kp_y;
Ki_y
];
	model.dstate=[];
	model.blocktype='d';
	model.dep_ut=[%t %f];
    exprs=[sci2exp(Kp_rp),sci2exp(Ki_rp),sci2exp(Kp_y),sci2exp(Ki_y)]
    gr_i=['xstringb(orig(1),orig(2),[''IMU ALG'' ],sz(1),sz(2),''fill'');'];
    x=standard_define([4 2],model,exprs,gr_i)
case 'readout' then
      BLOCK.version=020;
      BLOCK.name='imu_algorithm';
      BLOCK.comp_name='rt_imu_algorithm';
      BLOCK.desr_short='imu algorithm';
      BLOCK.dep_u=%t;
      BLOCK.dep_t=%f;
      BLOCK.blocktype='d';
      BLOCK.dstate='';
      BLOCK.IOmatrix=%f;
      BLOCK.inset=%f;
      BLOCK.in='9';
      BLOCK.outset=%f;
      BLOCK.out='[9;3;3]';
      BLOCK.evtin='1';
      BLOCK.evtout='';
      BLOCK.size='4 2';
      BLOCK.completelabel=%f;
      BLOCK.label=[39,39,73,77,85,32,65,76,71,39,39,10];
      BLOCK.ipar=[49,10];
      BLOCK.rpar=[75,112,95,114,112,59,10,75,105,95,114,112,59,10,75,112,95,121,59,10,75,105,95,121,10];
      BLOCK.opar=[];
      BLOCK.parameter=list();
      BLOCK.parameter($+1)=[];
      BLOCK.parameter(1).name='Kp_rp';
      BLOCK.parameter(1).text='Kp roll pitch:';
      BLOCK.parameter(1).type='vec';
      BLOCK.parameter(1).size='1';
      BLOCK.parameter(1).init='0.6';
      BLOCK.parameter(1).visible_plot=%f;
      BLOCK.parameter($+1)=[];
      BLOCK.parameter(2).name='Ki_rp';
      BLOCK.parameter(2).text='Kp roll pitch:';
      BLOCK.parameter(2).type='vec';
      BLOCK.parameter(2).size='1';
      BLOCK.parameter(2).init='0.00002';
      BLOCK.parameter(2).visible_plot=%f;
      BLOCK.parameter($+1)=[];
      BLOCK.parameter(3).name='Kp_y';
      BLOCK.parameter(3).text='Kp Yaw';
      BLOCK.parameter(3).type='vec';
      BLOCK.parameter(3).size='1';
      BLOCK.parameter(3).init='1.2';
      BLOCK.parameter(3).visible_plot=%f;
      BLOCK.parameter($+1)=[];
      BLOCK.parameter(4).name='Ki_y';
      BLOCK.parameter(4).text='Ki Yaw';
      BLOCK.parameter(4).type='vec';
      BLOCK.parameter(4).size='1';
      BLOCK.parameter(4).init='0.00002';
      BLOCK.parameter(4).visible_plot=%f;
      x=BLOCK;
  end
endfunction
function [x,y,typ] = read_razor_imu(job,arg1,arg2)
// Scicos Interface for razor IMU
//
// Block Screenshot
//
// Description
//
// Read out razor imu devices. Each block will start a thread that is continously
// reading out every datapacked send. The Scicos mail look will poll for this data.
// The Datarate is about 270 Hz for accel and gyro. For the magnetometer is about 270/20 Hz.
//
// Needed files:
//
// needs a directory razor_imu_calib in the CURRENT directory for calibration files.
// It should contain calib_<id>.dat for each sensor. Look at razor_imu/lib for 
// this calibration directory. Calibrations mey be created with razor_imu/lib/calibrate.sce.
//
// Note: it may take several seconds since this blocks gives valid outputs since 
// the firmware takes some time to initialise
// 
// Sensor ID:
// 
// Sensor id as printed on the devices
// /dev/razor_imu_<id> will appear, if the corresponding udev rules are installed
// 
// 
// Blocks outputs (like xsens_old):
// 
// - first port: port size is 9 containing:
//   - 1:3 acceleration 
//   - 4:6 angular velocities
//   - 7:9 magnetic field
// 
// 
//
//
//
//
//
//
//
//
//
//
// Interfacing Function
// read_razor_imu.sci
// Computitional Function
// razor_imu_hart.cpp
// Authors
// Christian Klauer
//


  x=[];y=[];typ=[];
  select job
  case 'plot' then
    exprs=arg1.graphics.exprs;
    standard_draw(arg1)
  case 'getinputs' then
    [x,y,typ]=standard_inputs(arg1)
  case 'getoutputs' then
    [x,y,typ]=standard_outputs(arg1)
  case 'getorigin' then
    [x,y]=standard_origin(arg1)
  case 'set' then
    x=arg1
    model=arg1.model;graphics=arg1.graphics;
    exprs=graphics.exprs;
    while %t do
  try
  getversion('scilab');
      [ok,sensorid,exprs]=..
      scicos_getvalue('Razor IMU',..
        ['Sensor ID:'],..
      list('vec',1),exprs)
catch
      [ok,sensorid,exprs]=..
      getvalue('Razor IMU',..
        ['Sensor ID:'],..
      list('vec',1),exprs)
end;
     if ~ok then break,end
      in=[model.in]
      out=[[9]]
      evtin=[1]
      evtout=[]
      [model,graphics,ok]=check_io(model,graphics,in,out,evtin,evtout);
      if ok then
        graphics.exprs=exprs;
        model.ipar=[1;
sensorid


];
        model.rpar=[];
   model.dstate=[1];
        x.graphics=graphics;x.model=model
        break
      end
    end
  case 'define' then
     sensorid=1;
   model=scicos_model()
   model.sim=list('rt_razor_imu',4)
   model.in=[[]]
   model.out=[[9]]
   model.evtin=[1]
   model.evtout=[]
   model.ipar=[1;
sensorid


];
   model.rpar=[];
 model.dstate=[1];
 model.blocktype='d';
 model.dep_ut=[%f %f];
    exprs=[sci2exp(sensorid)]
    gr_i=['xstringb(orig(1),orig(2),[''RAZOR IMU''   ],sz(1),sz(2),''fill'');'];
    x=standard_define([4 3],model,exprs,gr_i)
case 'readout' then
      BLOCK.version=020;
      BLOCK.name='read_razor_imu';
      BLOCK.comp_name='rt_razor_imu';
      BLOCK.desr_short='Razor IMU';
      BLOCK.dep_u=%f;
      BLOCK.dep_t=%f;
      BLOCK.blocktype='d';
      BLOCK.dstate='1';
      BLOCK.IOmatrix=%f;
      BLOCK.inset=%f;
      BLOCK.in='[]';
      BLOCK.outset=%t;
      BLOCK.out='[9]';
      BLOCK.evtin='1';
      BLOCK.evtout='';
      BLOCK.size='4 3';
      BLOCK.completelabel=%f;
      BLOCK.label=[39,39,82,65,90,79,82,32,73,77,85,39,39,10,10,10];
      BLOCK.ipar=[49,59,10,115,101,110,115,111,114,105,100,10,10,10];
      BLOCK.rpar=[];
      BLOCK.opar=[];
      BLOCK.parameter=list();
      BLOCK.parameter($+1)=[];
      BLOCK.parameter(1).name='sensorid';
      BLOCK.parameter(1).text='Sensor ID:';
      BLOCK.parameter(1).type='vec';
      BLOCK.parameter(1).size='1';
      BLOCK.parameter(1).init='1';
      BLOCK.parameter(1).visible_plot=%f;
      x=BLOCK;
  end
endfunction
// Interfacing functions are placed in this place

// function [sim,out] = ld_constmat(sim, events, mat) // PARSEDOCU_BLOCK
// // 
// // a constant matrix
// // 
// // mat *+ - the matrix
// // 
//   btype = 69001 + 1;
//   [nr, nc] = size(mat);
//   mat_length = nr*nc + 2;
// 
//   ipar = [mat_length; nr; nc; 0];
//   rpar = [nr,nc];
//   for i=1:nr,
//     rpar=cat(2,rpar, mat(i,:))
//   end
//   rpar=rpar';
// 
//   [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
//                    insizes=[1], outsizes=[ mat_length ], ...
//                    intypes=[ORTD.DATATYPE_FLOAT], outtypes=[ORTD.DATATYPE_FLOAT]  );
//  
//   [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
// endfunction
// 

// function [sim,out] = ld_constmat(sim, events, mat) // PARSEDOCU_BLOCK
// // 
// // a constant matrix
// // 
// // mat *+ - the matrix
// // 
// 
//   [nr, nc] = size(mat);
//   //rpar = [nr,nc];
//   rpar = [];
//   for i=1:nr,
//     rpar=cat(2,rpar, mat(i,:))
//   end
//   //rpar=rpar';
// 
//   [sim,out] = ld_constvec(sim, defaultevents, rpar );
// endfunction

function [sim,out] = ld_constmat(sim, events, mat) // PARSEDOCU_BLOCK
// 
// %PURPOSE: a constant matrix
// 
// mat *+ - the matrix
// 

//   [nr, nc] = size(mat);
//   //rpar = [nr,nc];
//   rpar = zeros(1,nr*nc);
//   for i=1:nr,
// //     rpar=cat(2,rpar, mat(i,:))
//     rpar( nc*(i-1)+1 : nc*i ) = mat(i,:);
//   end
//   //rpar=rpar';

  tmp = mat';

  [sim,out] = ld_constvec(sim, events, tmp(:)' );
endfunction

function [sim,out] = ld_matmul(sim, events, left_matrix, left_matrix_size, right_matrix, right_matrix_size) // PARSEDOCU_BLOCK
// %PURPOSE: matrix multiplication of two matrices
// 
// out = left_matrix * right_matrix 
//
//    left_matrix *+(left_matrix_size)  - matrix signal
//    left_matrix_size ---> matrix size [nr,nc]
//
//    right_matrix *+(right_matrix_size)  - matrix signal
//    right_matrix_size ---> matrix size [nr,nc]
//    
  btype = 69001 + 3;

  if (length(left_matrix_size) ~= 2)  then
    printf("ld_matmul: Incorrect size definition left_matrix_size\n");
    error(".");
  end
  if (length(right_matrix_size) ~= 2)  then
    printf("ld_matmul: Incorrect size definition right_matrix_size\n");
    error(".");
  end

  lm_nr = left_matrix_size(1);
  lm_nc = left_matrix_size(2);
  rm_nr = right_matrix_size(1);
  rm_nc = right_matrix_size(2);


  if (lm_nc ~= rm_nr)  then
    printf("ld_matmul: Inconsistent multiplication -> %dx%d * %dx%d!\n", lm_nr, lm_nc, rm_nr, rm_nc);
    error(".");
  end

  l_in_size = (lm_nr*lm_nc);
  r_in_size = (rm_nr*rm_nc);
  out_size  = (lm_nr*rm_nc);

  ipar = [lm_nr; lm_nc; rm_nr; rm_nc]; rpar = [];

  

  [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
                                     insizes=[l_in_size, r_in_size], outsizes=[out_size], ...
                                     intypes=[ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_FLOAT], outtypes=[ORTD.DATATYPE_FLOAT] );
 
  // libdyn_conn_equation connects multiple input signals to blocks
  [sim,blk] = libdyn_conn_equation(sim, blk, list( left_matrix, right_matrix ) );

  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

function [sim,y] = ld_matvecmul_3d(sim, events, A, v) // PARSEDOCU_BLOCK
// %PURPOSE: Matrix multiplication against a vector for 3d-vectors
// 
// y = A * v, whereby A is 3x3 and v is a vector of size 3
// 
// 
// 

  [sim,y] = ld_matmul(sim, events, A, [3,3], v, [3,1] );
endfunction

function [sim, v1, v2, v3] = ld_getvecelements_3d(sim, events, v);
// %PURPOSE: Return the elements of a 3d-vector
// 
// v = [v1, v2, v3]

  [sim,elements] = ld_demux(sim, events, vecsize=3, invec=v);
  
  v1 = elements(1); v2 = elements(2); v3 = elements(3);
endfunction

function [sim, v] = ld_setvecelements_3d(sim, events, v1, v2, v3);
// %PURPOSE: Combine three vector elements to a 3d-vector
// 
// v = [v1, v2, v3]

  [sim, v] = ld_mux(sim, events, vecsize=3, inlist=list( v1, v2, v3) );
endfunction

function [sim, out, param] = ld_leastsquares(sim, events, inplist, veclen, n_param) // PARSEDOCU_BLOCK
// %PURPOSE: estimate n linear model parameters (a,b,c...) and compute yvec_hat.
// 
// yvec_hat = a * phi0 + b * phi1 + c * phi2 ...
// Estimation of a, b, c ... minimizes the sum of squared errors (\chi^2 = \sum_i (yvec - yvec_hat)^2).
// out = yvec_hat
//
// inplist* - list(phi0*, phi1*, phi2*, ..., phi(n_param-1)*, yvec*)
// + phi0* - input vector0 (The vectors phiX form the matrix of predictor variables.)
// + phi1* - input vector1 (The vectors phiX form the matrix of predictor variables.)
// + phi2* - input vector2 (The vectors phiX form the matrix of predictor variables.)
// + ...
// + phi(n_param-1)* - input vector(n_param-1) (The vectors phiX form the matrix of predictor variables.)
// + yvec* - the original vector
//
// veclen - the number of observations which have been made (the length of each vector phiX, yvec and yvec_hat)
// n_param - the number of vectors belonging to Phi as well as the number of parameters in param*.
// out* = yvec_hat (the best prediction of yvec)
// param* - [a, b, c, ...]
//
// NOTICE: ld_leastsquares is not available by default since you need to include the GSL - GNU Scientific Library.
// Please follow the instructions which have been testet under Ubuntu 12.04. LTS:
//
// - install the required library with the following command or with a package-manager of your choice before installing ORTD:
// 'sudo apt-get install libgsl0-dev'
//
//    
  btype = 69001 + 4;

  ipar = [veclen, n_param]; rpar = [];

  if (veclen < n_param)
    error("ERROR: ld_leastsquares: Vectorsize must be greater or equal to the number of parameters to estimate.");
  end
  
  insizelist = [];
  intypelist = [];
  
  for j=1:(n_param + 1) // yvec is part of the list as well
    insizelist = [insizelist, veclen];
    intypelist = [intypelist, ORTD.DATATYPE_FLOAT];
  end
    

  [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
                                     insizes=insizelist, outsizes=[veclen, n_param], ...
                                     intypes=intypelist, ...
                                     outtypes=[ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_FLOAT] );
 
  // libdyn_conn_equation connects multiple input signals to blocks
  [sim,blk] = libdyn_conn_equation(sim, blk, inplist );

  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
  [sim,param] = libdyn_new_oport_hint(sim, blk, 1);   // 0th port
endfunction




//
//    Copyright (C) 2010, 2011  Christian Klauer
//
//    This file is part of OpenRTDynamics, the Real Time Dynamic Toolbox
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





// Interfacing functions are placed in this place


function [sim,bid] = libdyn_new_muparser(sim, events, Nin, Nout, str, float_param)
  btype = 11001;
  str = ascii(str);
  nparam = length(float_param);
  
  ipar = [Nin, Nout, nparam, length(str), str(:)'];
  rpar = [float_param(:)];
  
  [sim,bid] = libdyn_new_blk_generic(sim, events, btype, ipar, rpar);
endfunction

function [sim,out] = ld_muparser(sim, events, inlist, str, float_param) // PARSEDOCU_BLOCK
// 
// Evaluation of math formula
// 
// inlist - list() of signals of length 1
// str - string containing the formula
// float_param - vector of parameters
// 
// Within str "u1", "u2", ... refer to inputs and
//            "c1", "c2", ... refer to the parameters within float_param
// 
  Nin = length(inlist);
  Nout = 1;

  [sim,blk] = libdyn_new_muparser(sim, events, Nin, Nout, str, float_param);
  [sim,blk] = libdyn_conn_equation(sim, blk, inlist);
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction


//
// Use: http://help.scilab.org/docs/5.3.2/en_US/strsubst.html
//


//parNames = ['par1', 'par2']; par = [ 0.2. 0.4 ];
//inNames = ['u', 'e' ]; inlist = list(x,x);
//str = 'sin(par1) + par2';
//

function [sim,out] = ld_muparser_subst(sim, events, inlist, str, par, inNames, parNames) // PARSEDOCU_BLOCK
// 
// Bug within this scilab function
// 
// subtitution: haveing names like "phi" and "phi_u" does not work
//              names "u1", "u2", ... and "c1", "c2", ... also does not work right now
// 
// 
// 

  Nin = length(inlist);
  Nout = 1;
  
  inNames = inNames(:);
  parNames = parNames(:);
  
  [ tmp1, tmp2 ] = size(inNames);
  NinNames = max(tmp1, tmp2);

  [ tmp1, tmp2 ] = size(parNames);
  NparNames = max(tmp1, tmp2);

//printf("--------- BEGIN MUPARSER BLOCK -----------\n");  

//  pause;

  if length(inlist) ~= NinNames then
      printf("inNames and inlist have to have equal length (%d != %d)\n", length(inlist), NinNames);
      error(".");
  end
  if length(par) ~= NparNames then
      printf("parNames and par have to have equal length (%d != %d)\n", length(par), NparNames);
      error(".");
  end

  // substitute variable names for the inputs
  i = 1;
  while i <= Nin 
    if typeof(inNames(i)) ~= 'string' then
      error("ld_muparser2: Did not find a string in invar");
    end


    if libdyn_is_ldobject(inlist(i)) == %F then
      error("mu_parser2: Did not find a libdyn object at some entry within inlist");
    end


    // replace the variable name
    replacement = 'u' + string(i);
    //printf("subst %s with %s\n", inNames(i), replacement);
    str = strsubst( str, inNames(i), replacement );
    //printf("result is " + str + "\n");

    i = i + 1;
  end
  
  // substitute variable names for the parameters
  i = 1;
  while i <= length(par) 
    if typeof(parNames(i)) ~= 'string' then
      error("ld_muparser2: Did not find a sting in parNames");
    end


    if typeof(par(i)) ~= 'constant' then
      error("mu_parser2: Did not find a skalar");
    end

    replacement = 'c' + string(i);
  //  printf("subst %s with %s\n", parNames(i), replacement);
    str = strsubst( str, parNames(i), replacement );


    i = i + 1;
  end


//  printf("The expression is now: %s\n", str);


  [sim,out] = ld_muparser(sim, events, inlist, str, par);
endfunction


// 
// 
// 
// 
// 
// function [sim,out] = ld_muparser2(sim, events, inlist, str, invar, par)
//   // 
//   Nin = length(inlist);
//   Nout = 1;
// 
//   parlist = new_irparam_set();
// 
// 
//   inlist = list();
// 
//   n_in = 0;
//   i = 1;
//   //i_ = 1:length(invar);
//   while i <= length(invar) 
//     if typeof(invar(i)) ~= 'string' then
//       error("ld_muparser2: Did not find a sting in invar");
//     end
// 
//     parlist = new_irparam_elemet_ivec(parlist, invar(i), 100+n_in); // save name of input variable
// 
//     n_in = n_in + 1;
//     i = i + 1;
// 
// 
//     if libdyn_is_ldobject(invar(i)) == %F then
//       error("mu_parser2: Did not find a libdyn object in invar");
//     end
// 
//     inlist(n_in) = invar(i);
// 
//     i = i + 1;
// 
//   end
//   
// 
//   pnr = 0;
//   i = 1;
//   //i_ = 1:length(invar);
//   while i <= length(par) 
//     if typeof(par(i)) ~= 'string' then
//       error("ld_muparser2: Did not find a sting in invar");
//     end
// 
//     parlist = new_irparam_elemet_ivec(parlist, ascii(par(i)), 10000+pnr); // save name of parameter
// 
//     pnr = pnr + 1;
//     i = i + 1;
// 
// 
//     if  typeof(par(i)) ~= 'constant' then
//       error("mu_parser2: Did not find a skalar");
//     end
// 
//     parlist = new_irparam_elemet_rvec(parlist, par(i), 20000+pnr); // save parameter value
//     
// 
//     i = i + 1;
// 
//   end
// 
// 
//   // store expression
//   str = ascii(str);
//   parlist = new_irparam_elemet_ivec(parlist, str, 10); 
// 
//   // combine ir parameters
//   blockparam = combine_irparam(parlist);
// 
// 
//   // block parameters
//   ipar = [Nin, Nout, blockparam.ipar];
//   rpar = [blockparam.rpar];
// 
//   btype = 11002; // block type id
//   
//   [sim,bid] = libdyn_new_blk_generic(sim, events, btype, ipar, rpar);
// 
// 
//   [sim,blk] = libdyn_conn_equation(sim, blk, inlist); // connect all imputs
//   [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
// endfunction
// 
// 
function [sim, outlist, computation_finished, userdata] = ld_async_simulation(sim, ev, inlist, insizes, outsizes, intypes, outtypes, nested_fn, TriggerSignal, name, ThreadPrioStruct, userdata) // PARSEDOCU_BLOCK
    // 
    // %PURPOSE: Run a nested libdyn simulation within a a thread
    //
    // INPUT Signals: 
    //
    // TriggerSignal * - Trigger one simulation step of the threaded and nested simulation, if TriggerSignal == 1 
    //                
    // inlist - list( ) of input signals to the block, that will be forwarded to the nested simulation(s) (Note: Currently broken)
    //
    // PARAMETERS:
    // 
    // insizes - input ports configuration
    // outsizes - output ports configuration
    // intypes - ignored for now, put ORTD.DATATYPE_FLOAT for each port
    // outtypes - ignored for now, put ORTD.DATATYPE_FLOAT for each port
    // nested_fn - scilab function defining the sub-schematics
    //             The prototype must be: function [sim, outlist, userdata] = nested_fn(sim, inlist, userdata)
    // 
    // name (string) - the name of the nested simulation
    // ThreadPrioStruct - Properties of the thread. e.g.:
    // 	  ThreadPrioStruct.prio1=ORTD.ORTD_RT_NORMALTASK;
    // 	  ThreadPrioStruct.prio2=0, ThreadPrioStruct.cpu = -1;
    // 
    // userdata - A Scilab variable that will be forwarded to the function nested_fn
    // 
    // OUTPUTS:
    // 
    // outlist - list( ) of output signals
    // computation_finished - optional and only meanful if asynchron_simsteps > 0 (means async computation)
    // 
    // 6.8.14: Fixed Bug forwarding userdata
    // 

    // ThreadPrioStruct.prio1=0, ThreadPrioStruct.prio2=0, ThreadPrioStruct.cpu = 0;

    // check for sizes
    N1 = length(insizes);
    N2 = length(outsizes);
    N3 = length(intypes);
    N4 = length(outtypes);
    // ...
    if (length(inlist) ~= N1) then
        error("length inlist invalid or length of insizes invalid\n");
    end

    if N4 ~= N2 then
        error("length of outsizes invalid\n");
    end

    if N1 ~= N3 then
        error("length of intypes invalid\n");
    end

    Noutp = length(outsizes);

    // pack all parameters into a structure "parlist"
    parlist = new_irparam_set();

    // Create parameters



    parlist = new_irparam_elemet_ivec(parlist, insizes, 10); 
    parlist = new_irparam_elemet_ivec(parlist, outsizes, 11); 
    parlist = new_irparam_elemet_ivec(parlist, intypes, 12); 
    parlist = new_irparam_elemet_ivec(parlist, outtypes, 13); 

    //    parlist = new_irparam_elemet_ivec(parlist, [0, 0], 20); 
    parlist = new_irparam_elemet_ivec(parlist, ascii(name), 21); 

    //ThreadPrioStruct
    //        printf("Adding optional thread priority information\n");     
    //        ThreadPrioStruct = varargin(1);

    // also add thread priority
    try
        ThreadPrio = [ThreadPrioStruct.prio1, ThreadPrioStruct.prio2, ThreadPrioStruct.cpu];
    catch
        errstr="The structure for the thread''s priority ThreadPrioStruct is not ok. " + ...
        "It must contain at least the fields ThreadPrioStruct.prio1, ThreadPrioStruct.prio2, ThreadPrioStruct.cpu";
        //          errstr="The structure for the threads priority ThreadPrioStruct is not ok.";
        error(errstr);
        null;
    end
    parlist = new_irparam_elemet_ivec(parlist, ThreadPrio, 22); 


    // nested_fn -  the function to call for defining the schematic
    Nsimulations = 1; // create only one simulation  
    irpar_sim_idcounter = 900;

    for i = 1:Nsimulations
        // define schematic
        [sim_container_irpar, nested_sim, userdata] = libdyn_setup_sch2(nested_fn, insizes, outsizes,  intypes, outtypes, userdata);

        // pack simulations into irpar container with id = 901
        parlist = new_irparam_container(parlist, sim_container_irpar, irpar_sim_idcounter);

        // increase irpar_sim_idcounter so the next simulation gets another id
        irpar_sim_idcounter = irpar_sim_idcounter + 1;
    end


    // combine parameters  
    p = combine_irparam(parlist); // convert to two vectors of integers and floating point values respectively

    // Set-up the block parameters and I/O ports
    Uipar = [ p.ipar ];  Urpar = [ p.rpar ];
    btype = 15001 + 10; // Reference to the block's type (computational function). Use the same id you are giving via the "libdyn_compfnlist_add" C-function

    //   insizes=[1,1]; // Input port sizes
    //   outsizes=[1]; // Output port sizes
    insizes=[insizes(:)', 1];
    intypes=[intypes(:)', ORTD.DATATYPE_FLOAT]; // the additional trigger input signal
    outsizes=[outsizes(:)', 1];
    outtypes=[outtypes(:)', ORTD.DATATYPE_FLOAT]; // the addotional comp finished output signal

    disp(outsizes);

    dfeed=[1];  // for each output 0 (no df) or 1 (a direct feedthrough to one of the inputs)
    //   intypes=[ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_FLOAT]; // datatype for each input port
    //   outtypes=[ORTD.DATATYPE_FLOAT]; // datatype for each output port

    blocktype = 1; // 1-BLOCKTYPE_DYNAMIC (if block uses states), 2-BLOCKTYPE_STATIC (if there is only a static relationship between in- and output)

    // Create the block
    [sim, blk] = libdyn_CreateBlockAutoConfig(sim, ev, btype, blocktype, Uipar, Urpar, insizes, outsizes, intypes, outtypes, dfeed);

    //   // connect the inputs
    //  [sim,blk] = libdyn_conn_equation(sim, blk, list(in1, in2) ); // connect in1 to port 0 and in2 to port 1
    // 
    //   // connect the ouputs
    //  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port


    // add switch and reset input signals
    blocks_inlist = inlist;
    blocks_inlist($+1) = TriggerSignal; // add the trigger signal


    // connect all inputs
    [sim,blk] = libdyn_conn_equation(sim, blk, blocks_inlist );

    // connect all outputs
    outlist = list();
    if Noutp ~= 0 then
        for i = 0:(Noutp-1)
            [sim,out] = libdyn_new_oport_hint(sim, blk, i);   // ith port
            outlist(i+1) = out;
        end
    else
        null;
        //      printf("ld_simnest: No outputs to connect\n");
    end

    // connect the computation finished indication signal
    [sim,computation_finished] = libdyn_new_oport_hint(sim, blk, Noutp);   // the last port


endfunction







function [sim, outlist, userdata] = ld_NoResetNest(sim, ev, inlist, insizes, outsizes, intypes, outtypes, nested_fn, ResetLevel, SimnestName, userdata) // PARSEDOCU_BLOCK
    // 
    // %PURPOSE: Run a nested libdyn simulation and prevent resets of this simulation
    //
    // INPUT Signals: 
    //
    // inlist - list( ) of input signals to the block, that will be forwarded to the nested simulation(s) (Note: Currently broken)
    //
    // PARAMETERS:
    // 
    // insizes - input ports configuration
    // outsizes - output ports configuration
    // intypes - ignored for now, put ORTD.DATATYPE_FLOAT for each port
    // outtypes - ignored for now, put ORTD.DATATYPE_FLOAT for each port
    // nested_fn - scilab function defining the sub-schematics
    //             The prototype must be: function [sim, outlist, userdata] = nested_fn(sim, inlist, userdata)
    // 
    // ResetLevel - set to -1
    // SimnestName (string) - the name of the nested simulation
    // 
    // userdata - A Scilab variable that will be forwarded to the function nested_fn
    // 
    // OUTPUTS:
    // 
    // outlist - list( ) of output signals
    // 


    // check for sizes
    N1 = length(insizes);
    N2 = length(outsizes);
    N3 = length(intypes);
    N4 = length(outtypes);
    // ...
    if (length(inlist) ~= N1) then
        error("length inlist invalid or length of insizes invalid\n");
    end

    if N4 ~= N2 then
        error("length of outsizes invalid\n");
    end

    if N1 ~= N3 then
        error("length of intypes invalid\n");
    end

    Noutp = length(outsizes);

    // pack all parameters into a structure "parlist"
    parlist = new_irparam_set();

    // Create parameters
    parlist = new_irparam_elemet_ivec(parlist, insizes, 10); 
    parlist = new_irparam_elemet_ivec(parlist, outsizes, 11); 
    parlist = new_irparam_elemet_ivec(parlist, intypes, 12); 
    parlist = new_irparam_elemet_ivec(parlist, outtypes, 13); 

    //    parlist = new_irparam_elemet_ivec(parlist, [0, 0], 20); 
    parlist = new_irparam_elemet_ivec(parlist, ascii(SimnestName), 21); 

    // nested_fn -  the function to call for defining the schematic
    Nsimulations = 1; // create only one simulation  
    irpar_sim_idcounter = 900;

    for i = 1:Nsimulations
        // define schematic
        [sim_container_irpar, nested_sim, TMPuserdata] = libdyn_setup_sch2(nested_fn, insizes, outsizes,  intypes, outtypes, list(i, userdata));
        userdata = TMPuserdata(2);

        // pack simulations into irpar container with id = 901
        parlist = new_irparam_container(parlist, sim_container_irpar, irpar_sim_idcounter);

        // increase irpar_sim_idcounter so the next simulation gets another id
        irpar_sim_idcounter = irpar_sim_idcounter + 1;
    end


    // combine parameters  
    p = combine_irparam(parlist); // convert to two vectors of integers and floating point values respectively

    // Set-up the block parameters and I/O ports
    Uipar = [ p.ipar ];  Urpar = [ p.rpar ];
    btype = 15001 + 11; // Reference to the block's type (computational function). Use the same id you are giving via the "libdyn_compfnlist_add" C-function

    //   // add additional input
    //   insizes=[insizes(:)', 1];
    //   intypes=[intypes(:)', ORTD.DATATYPE_FLOAT]; // the additional trigger input signal
    // 
    //   // add additional output
    //   outsizes=[outsizes(:)', 1];
    //   outtypes=[outtypes(:)', ORTD.DATATYPE_FLOAT]; // the addotional comp finished output signal

    //   disp(outsizes);

    dfeed=[1];  // for each output 0 (no df) or 1 (a direct feedthrough to one of the inputs)
    blocktype = 1; // 1-BLOCKTYPE_DYNAMIC (if block uses states), 2-BLOCKTYPE_STATIC (if there is only a static relationship between in- and output)

    // Create the block
    [sim, blk] = libdyn_CreateBlockAutoConfig(sim, ev, btype, blocktype, Uipar, Urpar, insizes, outsizes, intypes, outtypes, dfeed);

    // add switch and reset input signals
    blocks_inlist = inlist;

    // add an additional input signal
    //   blocks_inlist($+1) = TriggerSignal; 

    // connect all inputs
    [sim,blk] = libdyn_conn_equation(sim, blk, blocks_inlist );

    // connect all outputs
    outlist = list();
    if Noutp ~= 0 then
        for i = 0:(Noutp-1)
            [sim,out] = libdyn_new_oport_hint(sim, blk, i);   // ith port
            outlist(i+1) = out;
        end
    else
        null;
        //      printf("ld_simnest: No outputs to connect\n");
    end

    // connect additional outputs
    //   [sim,computation_finished] = libdyn_new_oport_hint(sim, blk, Noutp+0);   // the last port


endfunction



// 
// Use this as a template for nesting simulations
// 

function [sim, outlist, userdata] = ld_CaseSwitchNest(sim, ev, inlist, insizes, outsizes, intypes, outtypes, CaseSwitch_fn, SimnestName, DirectFeedthrough, SelectSignal, CaseNameList, userdata) // PARSEDOCU_BLOCK
    // 
    // %PURPOSE: Switch mechanism for multiple nested simulations 
    //
    // INPUT Signals: 
    //
    // inlist - list( ) of input signals to the block, that will be forwarded to the nested simulation(s) (Note: Currently broken)
    // SelectSignal - * swicht signal of type ORTD.DATATYPE_INT32
    //
    // PARAMETERS:
    // 
    // insizes - input ports configuration
    // outsizes - output ports configuration
    // intypes - ignored for now, put ORTD.DATATYPE_FLOAT for each port
    // outtypes - ignored for now, put ORTD.DATATYPE_FLOAT for each port
    // CaseSwitch_fn - scilab function defining the sub-schematics
    //             The prototype must be: function [sim, outlist, userdata] = nested_fn(sim, inlist, Ncase, casename, userdata)
    // 
    // SimnestName (string) - the name of the nested simulation
    // DirectFeedthrough - %t or %f
    // SelectSignal * - int32 use to determine the currently active simulation
    // CaseNameList list() of strings
    // 
    // userdata - A Scilab variable that will be forwarded to the function nested_fn
    // 
    // OUTPUTS:
    // 
    // outlist - list( ) of output signals
    //
    // PLEASE NOTE: For ld_CaseSwitchNest at least one output must be defined such that
    //              the nested simulations are executed at the right time instances.
    //              Otherwise a delayed execution of the nested simulations has been oberserved.
    // 



    function [sim, outlist, userdata ] = WrapCaseSwitch(sim, inlist, userdata)
        Ncase = userdata(1);
        userdata_nested = userdata(2);
        fn = userdata(3);
        casename = userdata(4);  
        Nin_usersignals = userdata(5);
        Nout_usersignals = userdata(6);

        printf("ld_CaseSwitchNest: case=%s (#%d)\n", casename, Ncase );

        // make a list containing only the input signals forwarded to the nested simulation(s)
        inlist_inner = list();     
        for i = 1:Nin_usersignals
            inlist_inner($+1) = inlist(i);
        end

        //pause;

        // additional input signals
        //     additionalInput1 = inlist(N+1);

        // call the actual function
        [sim, outlist_inner, userdata_nested] = fn(sim, inlist_inner, Ncase, casename, userdata_nested);

        if length(outlist_inner) ~= Nout_usersignals then
            printf("ld_CaseSwitchNest: your provided schmatic-describing function returns more or less outputs in outlist. Expecting %d but there are %d\n", Nout_usersignals, length(outlist_inner));
            error(".");
        end

        // make a list containing only the output signals comming form the nested simulation(s)
        outlist = list();
        N = length(outlist_inner);
        for i = 1:N
            outlist($+1) = outlist_inner(i);
        end
        // additional outputs
        //     outlist($+1) = active_state;

        userdata = list( userdata_nested ); // additional data that should be returned can be added here
    endfunction

    //  TODO go further form here on
    Nsimulations = length(CaseNameList); //

    // check for sizes
    N1 = length(insizes);
    N2 = length(outsizes);
    N3 = length(intypes);
    N4 = length(outtypes);
    // ...
    if (length(inlist) ~= N1) then
        error("length inlist invalid or length of insizes invalid\n");
    end

    if N4 ~= N2 then
        error("length of outsizes invalid\n");
    end

    if N1 ~= N3 then
        error("length of intypes invalid\n");
    end

    if DirectFeedthrough then
        dfeed=[1];
    else
        dfeed=[0];
    end

    Noutp = length(outsizes);

    // pack all parameters into a structure "parlist"
    parlist = new_irparam_set();

    // Create parameters


    parlist = new_irparam_elemet_ivec(parlist, [Nsimulations, dfeed], 1); // general parameters in a list
    parlist = new_irparam_elemet_ivec(parlist, insizes, 10); 
    parlist = new_irparam_elemet_ivec(parlist, outsizes, 11); 
    parlist = new_irparam_elemet_ivec(parlist, intypes, 12); 
    parlist = new_irparam_elemet_ivec(parlist, outtypes, 13); 

    //    parlist = new_irparam_elemet_ivec(parlist, [0, 0], 20); 
    parlist = new_irparam_elemet_ivec(parlist, ascii(SimnestName), 21); 


    irpar_sim_idcounter = 900;
    for i = 1:Nsimulations

        Ncase = i;  casename = CaseNameList(Ncase);
        userdata__ = list( Ncase, userdata, CaseSwitch_fn, casename, length(insizes), length(outsizes) );

        // define schematic
        nested_fn__ = WrapCaseSwitch; // nested_fn -  the function to call for defining the schematic
        [sim_container_irpar, nested_sim, TMPuserdata] = libdyn_setup_sch2(nested_fn__, insizes, outsizes, intypes, outtypes, userdata__);
        userdata = TMPuserdata(1);

        // pack simulations into irpar container with id = 901
        parlist = new_irparam_container(parlist, sim_container_irpar, irpar_sim_idcounter);

        // increase irpar_sim_idcounter so the next simulation gets another id
        irpar_sim_idcounter = irpar_sim_idcounter + 1;
    end


    // combine parameters  
    p = combine_irparam(parlist); // convert to two vectors of integers and floating point values respectively

    // Set-up the block parameters and I/O ports
    Uipar = [ p.ipar ];  Urpar = [ p.rpar ];
    btype = 15001 + 12; // Reference to the block's type (computational function). Use the same id you are giving via the "libdyn_compfnlist_add" C-function

    // add additional input(s)
    insizes=[insizes(:)', 1];
    intypes=[intypes(:)', ORTD.DATATYPE_INT32]; // the additional switch input signal
    // 
    //   // add additional output
    //   outsizes=[outsizes(:)', 1];
    //   outtypes=[outtypes(:)', ORTD.DATATYPE_FLOAT]; // the addotional comp finished output signal

    //   disp(outsizes);


    blocktype = 1; // 1-BLOCKTYPE_DYNAMIC (if block uses states), 2-BLOCKTYPE_STATIC (if there is only a static relationship between in- and output)

    // Create the block
    [sim, blk] = libdyn_CreateBlockAutoConfig(sim, ev, btype, blocktype, Uipar, Urpar, insizes, outsizes, intypes, outtypes, dfeed);

    // add switch and reset input signals
    blocks_inlist = inlist;

    // add an additional input signal
    blocks_inlist($+1) = SelectSignal; 


    // connect all inputs
    [sim,blk] = libdyn_conn_equation(sim, blk, blocks_inlist );

    // connect all outputs and put them into the outlist-list
    outlist = list();
    if Noutp ~= 0 then
        for i = 0:(Noutp-1)
            [sim,out] = libdyn_new_oport_hint(sim, blk, i);   // ith port
            outlist(i+1) = out;
        end
    else
        null;
        //      printf("ld_simnest: No outputs to connect\n");
    end

    // connect additional outputs
    //   [sim,computation_finished] = libdyn_new_oport_hint(sim, blk, Noutp+0);   // the last port


endfunction



function [sim, outlist, userdata] = ld_ForLoopNest(sim, ev, inlist, insizes, outsizes, intypes, outtypes, ForLoop_fn, SimnestName, NitSignal, userdata) 
    // PARSEDOCU_BLOCK
    // 
    // %PURPOSE: Switch mechanism for multiple nested simulations 
    //
    // INPUT Signals: 
    //
    // inlist - list( ) of input signals to the block, that will be forwarded to the nested simulation(s) (Note: Currently broken)
    // SelectSignal - * swicht signal of type ORTD.DATATYPE_INT32
    // NitSignal * - int32 The number of simulation steps to be performed by the nested simulation
    //
    // PARAMETERS:
    // 
    // insizes - input ports configuration
    // outsizes - output ports configuration
    // intypes - ignored for now, put ORTD.DATATYPE_FLOAT for each port
    // outtypes - ignored for now, put ORTD.DATATYPE_FLOAT for each port
    // ForLoop_fn - scilab function defining the sub-schematics
    //             The prototype must be: function [sim, outlist, userdata] = nested_fn(sim, inlist, CounterSignal, userdata)
    // 
    // SimnestName (string) - the name of the nested simulation
    // 
    // userdata - A Scilab variable that will be forwarded to the function nested_fn
    // 
    // OUTPUTS:
    // 
    // outlist - list( ) of output signals
    // 


    function [sim, outlist, userdata ] = WrapForLoop(sim, inlist, userdata)
        userdata_nested = userdata(1);
        fn = userdata(2);
        Nin_usersignals = userdata(3);
        Nout_usersignals = userdata(4);

//        printf("ld_CaseSwitchNest: case=%s (#%d)\n", casename, Ncase );

        // make a list containing only the input signals forwarded to the nested simulation(s)
        inlist_inner = list();     
        for i = 1:Nin_usersignals
            inlist_inner($+1) = inlist(i);
        end

        // additional input signals to the nested simulation
        CounterSignal = inlist(Nin_usersignals+1);


        // call the actual function
        [sim, outlist_inner, userdata_nested] = fn(sim, inlist_inner, CounterSignal, userdata_nested);

        if length(outlist_inner) ~= Nout_usersignals then
            printf("ld_ForLoopNest: your provided schmatic-describing function returns more or less outputs in outlist. Expecting %d but there are %d\n", Nout_usersignals, length(outlist_inner));
            error(".");
        end

        // make a list containing only the output signals comming form the nested simulation(s)
        outlist = list();
        N = length(outlist_inner);
        for i = 1:N
            outlist($+1) = outlist_inner(i);
        end
        // additional outputs
        //     outlist($+1) = active_state;

        userdata = list( userdata_nested ); // additional data that should be returned can be added here
    endfunction

    Nsimulations = 1; //

    // check for sizes
    N1 = length(insizes);
    N2 = length(outsizes);
    N3 = length(intypes);
    N4 = length(outtypes);
    // ...
    if (length(inlist) ~= N1) then
        error("length inlist invalid or length of insizes invalid\n");
    end

    if N4 ~= N2 then
        error("length of outsizes invalid\n");
    end

    if N1 ~= N3 then
        error("length of intypes invalid\n");
    end

    Noutp = length(outsizes);

    // pack all parameters into a structure "parlist"
    parlist = new_irparam_set();

    // Create parameters

dfeed = 1;

    parlist = new_irparam_elemet_ivec(parlist, [dfeed ], 1); // general parameters in a list
    parlist = new_irparam_elemet_ivec(parlist, insizes, 10); 
    parlist = new_irparam_elemet_ivec(parlist, outsizes, 11); 
    parlist = new_irparam_elemet_ivec(parlist, intypes, 12); 
    parlist = new_irparam_elemet_ivec(parlist, outtypes, 13); 

    //    parlist = new_irparam_elemet_ivec(parlist, [0, 0], 20); 
    parlist = new_irparam_elemet_ivec(parlist, ascii(SimnestName), 21); 


    // add additional inputs to the nested simulation
    insizes_ToNest = [insizes(:); 1]; // one additional input for the loop counter
    intypes_ToNest = [intypes(:); ORTD.DATATYPE_INT32 ];

    // add additional outputs comming from the nested simulations
//    outsizes_FromNest = [outsizes(:); 1]; // one additional output for ...
//    outtypes_FromNest = [outtypes(:); ORTD.DATATYPE_INT32 ];
    outsizes_FromNest = [outsizes(:)  ]; 
    outtypes_FromNest = [outtypes(:)  ];
    

    irpar_sim_idcounter = 900;
    for i = 1:Nsimulations
        
        userdata__ = list( userdata, ForLoop_fn, length(insizes), length(outsizes) )

        // define schematic
        nested_fn__ = WrapForLoop; // nested_fn -  the function to call for defining the schematic
        [sim_container_irpar, nested_sim, TMPuserdata] = libdyn_setup_sch2(nested_fn__, insizes_ToNest, outsizes_FromNest, insizes_ToNest, outtypes_FromNest, userdata__);
        userdata = TMPuserdata(1);

        // pack simulations into irpar container with id = 901
        parlist = new_irparam_container(parlist, sim_container_irpar, irpar_sim_idcounter);

        // increase irpar_sim_idcounter so the next simulation gets another id
        irpar_sim_idcounter = irpar_sim_idcounter + 1;
    end


    // combine parameters  
    p = combine_irparam(parlist); // convert to two vectors of integers and floating point values respectively

    // Set-up the block parameters and I/O ports
    Uipar = [ p.ipar ];  Urpar = [ p.rpar ];
    dfeed = 1;
    btype = 15001 + 13; // Reference to the block's type (computational function). Use the same id you are giving via the "libdyn_compfnlist_add" C-function

    // add additional input(s) to the block containing the nested simulation(s)
    insizes=[insizes(:)', 1];
    intypes=[intypes(:)', ORTD.DATATYPE_INT32]; // the additional switch input signal
    // 
    //   // add additional outputs
    //   outsizes=[outsizes(:)', 1];
    //   outtypes=[outtypes(:)', ORTD.DATATYPE_FLOAT]; // the addotional comp finished output signal

    //   disp(outsizes);


    blocktype = 1; // 1-BLOCKTYPE_DYNAMIC (if block uses states), 2-BLOCKTYPE_STATIC (if there is only a static relationship between in- and output)

    // Create the block
    [sim, blk] = libdyn_CreateBlockAutoConfig(sim, ev, btype, blocktype, Uipar, Urpar, insizes, outsizes, intypes, outtypes, dfeed);

    // make a list of input signals to the block containing the nested simulation(s)
    blocks_inlist = inlist;

    // add an additional input signal
    blocks_inlist($+1) = NitSignal; 

    // connect all inputs
    [sim,blk] = libdyn_conn_equation(sim, blk, blocks_inlist );

    // connect all outputs and put them into the outlist-list
    outlist = list();
    if Noutp ~= 0 then
        for i = 0:(Noutp-1)
            [sim,out] = libdyn_new_oport_hint(sim, blk, i);   // ith port
            outlist(i+1) = out;
        end
    else
        null;
        //      printf("ld_simnest: No outputs to connect\n");
    end

    // connect additional outputs
    //   [sim,computation_finished] = libdyn_new_oport_hint(sim, blk, Noutp+0);   // the last port


endfunction


//
// Uses same C-fn like ld_ForLoopNest
//
function [sim, outlist, userdata] = ld_ForLoopNest2(sim, ev, inlist, insizes, outsizes, intypes, outtypes, ForLoop_fn, SimnestName, ResetAfterLoop, NitSignal, userdata) 
    // PARSEDOCU_BLOCK
    // 
    // %PURPOSE: Switch mechanism for multiple nested simulations 
    //
    // INPUT Signals: 
    //
    // inlist - list( ) of input signals to the block, that will be forwarded to the nested simulation(s) (Note: Currently broken)
    // SelectSignal - * swicht signal of type ORTD.DATATYPE_INT32
    // NitSignal * - int32 The number of simulation steps to be performed by the nested simulation
    //
    // PARAMETERS:
    // 
    // insizes - input ports configuration
    // outsizes - output ports configuration
    // intypes - ignored for now, put ORTD.DATATYPE_FLOAT for each port
    // outtypes - ignored for now, put ORTD.DATATYPE_FLOAT for each port
    // ForLoop_fn - scilab function defining the sub-schematics
    //             The prototype must be: function [sim, outlist, userdata] = nested_fn(sim, inlist, CounterSignal, userdata)
    // 
    // SimnestName (string) - the name of the nested simulation
    // ResetAfterLoop - 1 or 0 Reset the nested simulation after the loop has finished if ResetAfterLoop == 1
    // 
    // userdata - A Scilab variable that will be forwarded to the function nested_fn
    // 
    // OUTPUTS:
    // 
    // outlist - list( ) of output signals
    // 


    function [sim, outlist, userdata ] = WrapForLoop(sim, inlist, userdata)
        userdata_nested = userdata(1);
        fn = userdata(2);
        Nin_usersignals = userdata(3);
        Nout_usersignals = userdata(4);

//        printf("ld_CaseSwitchNest: case=%s (#%d)\n", casename, Ncase );

        // make a list containing only the input signals forwarded to the nested simulation(s)
        inlist_inner = list();     
        for i = 1:Nin_usersignals
            inlist_inner($+1) = inlist(i);
        end

        // additional input signals to the nested simulation
        CounterSignal = inlist(Nin_usersignals+1);


        // call the actual function
        [sim, outlist_inner, userdata_nested] = fn(sim, inlist_inner, CounterSignal, userdata_nested);

        if length(outlist_inner) ~= Nout_usersignals then
            printf("ld_ForLoopNest: your provided schmatic-describing function returns more or less outputs in outlist. Expecting %d but there are %d\n", Nout_usersignals, length(outlist_inner));
            error(".");
        end

        // make a list containing only the output signals comming form the nested simulation(s)
        outlist = list();
        N = length(outlist_inner);
        for i = 1:N
            outlist($+1) = outlist_inner(i);
        end
        // additional outputs
        //     outlist($+1) = active_state;

        userdata = list( userdata_nested ); // additional data that should be returned can be added here
    endfunction

    Nsimulations = 1; //

    // check for sizes
    N1 = length(insizes);
    N2 = length(outsizes);
    N3 = length(intypes);
    N4 = length(outtypes);
    // ...
    if (length(inlist) ~= N1) then
        error("length inlist invalid or length of insizes invalid\n");
    end

    if N4 ~= N2 then
        error("length of outsizes invalid\n");
    end

    if N1 ~= N3 then
        error("length of intypes invalid\n");
    end

    Noutp = length(outsizes);

    // pack all parameters into a structure "parlist"
    parlist = new_irparam_set();

    // Create parameters
    dfeed = 1;

    parlist = new_irparam_elemet_ivec(parlist, [dfeed, ResetAfterLoop ], 1); // general parameters in a list
    parlist = new_irparam_elemet_ivec(parlist, insizes, 10); 
    parlist = new_irparam_elemet_ivec(parlist, outsizes, 11); 
    parlist = new_irparam_elemet_ivec(parlist, intypes, 12); 
    parlist = new_irparam_elemet_ivec(parlist, outtypes, 13); 

    //    parlist = new_irparam_elemet_ivec(parlist, [0, 0], 20); 
    parlist = new_irparam_elemet_ivec(parlist, ascii(SimnestName), 21); 


    // add additional inputs to the nested simulation
    insizes_ToNest = [insizes(:); 1]; // one additional input for the loop counter
    intypes_ToNest = [intypes(:); ORTD.DATATYPE_INT32 ];

    // add additional outputs comming from the nested simulations
//    outsizes_FromNest = [outsizes(:); 1]; // one additional output for ...
//    outtypes_FromNest = [outtypes(:); ORTD.DATATYPE_INT32 ];
    outsizes_FromNest = [outsizes(:)  ]; 
    outtypes_FromNest = [outtypes(:)  ];
    

    irpar_sim_idcounter = 900;
    for i = 1:Nsimulations
        
        userdata__ = list( userdata, ForLoop_fn, length(insizes), length(outsizes) )

        // define schematic
        nested_fn__ = WrapForLoop; // nested_fn -  the function to call for defining the schematic
        [sim_container_irpar, nested_sim, TMPuserdata] = libdyn_setup_sch2(nested_fn__, insizes_ToNest, outsizes_FromNest, insizes_ToNest, outtypes_FromNest, userdata__);
        userdata = TMPuserdata(1);

        // pack simulations into irpar container with id = 901
        parlist = new_irparam_container(parlist, sim_container_irpar, irpar_sim_idcounter);

        // increase irpar_sim_idcounter so the next simulation gets another id
        irpar_sim_idcounter = irpar_sim_idcounter + 1;
    end


    // combine parameters  
    p = combine_irparam(parlist); // convert to two vectors of integers and floating point values respectively

    // Set-up the block parameters and I/O ports
    Uipar = [ p.ipar ];  Urpar = [ p.rpar ];
    dfeed = 1;
    btype = 15001 + 13; // Reference to the block's type (computational function). Use the same id you are giving via the "libdyn_compfnlist_add" C-function

    // add additional input(s) to the block containing the nested simulation(s)
    insizes=[insizes(:)', 1];
    intypes=[intypes(:)', ORTD.DATATYPE_INT32]; // the additional switch input signal
    // 
    //   // add additional outputs
    //   outsizes=[outsizes(:)', 1];
    //   outtypes=[outtypes(:)', ORTD.DATATYPE_FLOAT]; // the addotional comp finished output signal

    //   disp(outsizes);


    blocktype = 1; // 1-BLOCKTYPE_DYNAMIC (if block uses states), 2-BLOCKTYPE_STATIC (if there is only a static relationship between in- and output)

    // Create the block
    [sim, blk] = libdyn_CreateBlockAutoConfig(sim, ev, btype, blocktype, Uipar, Urpar, insizes, outsizes, intypes, outtypes, dfeed);

    // make a list of input signals to the block containing the nested simulation(s)
    blocks_inlist = inlist;

    // add an additional input signal
    blocks_inlist($+1) = NitSignal; 

    // connect all inputs
    [sim,blk] = libdyn_conn_equation(sim, blk, blocks_inlist );

    // connect all outputs and put them into the outlist-list
    outlist = list();
    if Noutp ~= 0 then
        for i = 0:(Noutp-1)
            [sim,out] = libdyn_new_oport_hint(sim, blk, i);   // ith port
            outlist(i+1) = out;
        end
    else
        null;
        //      printf("ld_simnest: No outputs to connect\n");
    end

    // connect additional outputs
    //   [sim,computation_finished] = libdyn_new_oport_hint(sim, blk, Noutp+0);   // the last port


endfunction





// NEWNEWNEW Adapted from ld_ForLoopNest, Okay delete this again
function [sim, outlist, userdata] = ld_TriggeredNest(sim, ev, inlist, insizes, outsizes, intypes, outtypes, nested_fn, SimnestName, TiggerSignal, userdata) 
    // PARSEDOCU_BLOCK
    // 
    // %PURPOSE: Switch mechanism for multiple nested simulations 
    //
    // INPUT Signals: 
    //
    // inlist - list( ) of input signals to the block, that will be forwarded to the nested simulation(s) (Note: Currently broken)
    // SelectSignal - * swicht signal of type ORTD.DATATYPE_INT32
    //
    // PARAMETERS:
    // 
    // insizes - input ports configuration
    // outsizes - output ports configuration
    // intypes - ignored for now, put ORTD.DATATYPE_FLOAT for each port
    // outtypes - ignored for now, put ORTD.DATATYPE_FLOAT for each port
    // nested_fn - scilab function defining the sub-schematics
    //             The prototype must be: function [sim, outlist, userdata] = nested_fn(sim, inlist, CounterSignal, userdata)
    // 
    // SimnestName (string) - the name of the nested simulation
    // TiggerSignal * - int32 To tigger one execution step of the nested simulation
    // 
    // userdata - A Scilab variable that will be forwarded to the function nested_fn
    // 
    // OUTPUTS:
    // 
    // outlist - list( ) of output signals
    // 


    function [sim, outlist, userdata ] = WrapForLoop(sim, inlist, userdata)
        userdata_nested = userdata(1);
        fn = userdata(2);
        Nin_usersignals = userdata(3);
        Nout_usersignals = userdata(4);

//        printf("ld_CaseSwitchNest: case=%s (#%d)\n", casename, Ncase );

        // make a list containing only the input signals forwarded to the nested simulation(s)
        inlist_inner = list();     
        for i = 1:Nin_usersignals
            inlist_inner($+1) = inlist(i);
        end

        // additional input signals to the nested simulation
        // CounterSignal = inlist(Nin_usersignals+1);


        // call the actual function
        [sim, outlist_inner, userdata_nested] = fn(sim, inlist_inner, userdata_nested);

        if length(outlist_inner) ~= Nout_usersignals then
            printf("ld_TriggeredNest: your provided schmatic-describing function returns more or less outputs in outlist. Expecting %d but there are %d\n", Nout_usersignals, length(outlist_inner));
            error(".");
        end

        // make a list containing only the output signals comming form the nested simulation(s)
        outlist = list();
        N = length(outlist_inner);
        for i = 1:N
            outlist($+1) = outlist_inner(i);
        end
        // additional outputs
        //     outlist($+1) = active_state;

        userdata = list( userdata_nested ); // additional data that should be returned can be added here
    endfunction

    Nsimulations = 1; //

    // check for sizes
    N1 = length(insizes);
    N2 = length(outsizes);
    N3 = length(intypes);
    N4 = length(outtypes);
    // ...
    if (length(inlist) ~= N1) then
        error("length inlist invalid or length of insizes invalid\n");
    end

    if N4 ~= N2 then
        error("length of outsizes invalid\n");
    end

    if N1 ~= N3 then
        error("length of intypes invalid\n");
    end

    Noutp = length(outsizes);

    // pack all parameters into a structure "parlist"
    parlist = new_irparam_set();

    // Create parameters

dfeed = 1;

    parlist = new_irparam_elemet_ivec(parlist, [dfeed ], 1); // general parameters in a list
    parlist = new_irparam_elemet_ivec(parlist, insizes, 10); 
    parlist = new_irparam_elemet_ivec(parlist, outsizes, 11); 
    parlist = new_irparam_elemet_ivec(parlist, intypes, 12); 
    parlist = new_irparam_elemet_ivec(parlist, outtypes, 13); 

    //    parlist = new_irparam_elemet_ivec(parlist, [0, 0], 20); 
    parlist = new_irparam_elemet_ivec(parlist, ascii(SimnestName), 21); 


    // add additional inputs to the nested simulation
    insizes_ToNest = [insizes(:) ]; 
    intypes_ToNest = [intypes(:) ];

    // add additional outputs comming from the nested simulations
//    outsizes_FromNest = [outsizes(:); 1]; // one additional output for ...
//    outtypes_FromNest = [outtypes(:); ORTD.DATATYPE_INT32 ];
    outsizes_FromNest = [outsizes(:)  ]; 
    outtypes_FromNest = [outtypes(:)  ];
    

    irpar_sim_idcounter = 900;
    for i = 1:Nsimulations
        
        userdata__ = list( userdata, nested_fn, length(insizes), length(outsizes) )

        // define schematic
        nested_fn__ = WrapForLoop; // nested_fn -  the function to call for defining the schematic
        [sim_container_irpar, nested_sim, TMPuserdata] = libdyn_setup_sch2(nested_fn__, insizes_ToNest, outsizes_FromNest, insizes_ToNest, outtypes_FromNest, userdata__);
        userdata = TMPuserdata(1);

        // pack simulations into irpar container with id = 901
        parlist = new_irparam_container(parlist, sim_container_irpar, irpar_sim_idcounter);

        // increase irpar_sim_idcounter so the next simulation gets another id
        irpar_sim_idcounter = irpar_sim_idcounter + 1;
    end


    // combine parameters  
    p = combine_irparam(parlist); // convert to two vectors of integers and floating point values respectively

    // Set-up the block parameters and I/O ports
    Uipar = [ p.ipar ];  Urpar = [ p.rpar ];
    dfeed = 1;
    btype = 15001 + 13; // Reference to the block's type (computational function). Use the same id you are giving via the "libdyn_compfnlist_add" C-function

    // add additional input(s) to the block containing the nested simulation(s)
    insizes=[insizes(:)', 1];
    intypes=[intypes(:)', ORTD.DATATYPE_INT32]; // the additional switch input signal
    // 
    //   // add additional outputs
    //   outsizes=[outsizes(:)', 1];
    //   outtypes=[outtypes(:)', ORTD.DATATYPE_FLOAT]; // the addotional comp finished output signal

    //   disp(outsizes);


    blocktype = 1; // 1-BLOCKTYPE_DYNAMIC (if block uses states), 2-BLOCKTYPE_STATIC (if there is only a static relationship between in- and output)

    // Create the block
    [sim, blk] = libdyn_CreateBlockAutoConfig(sim, ev, btype, blocktype, Uipar, Urpar, insizes, outsizes, intypes, outtypes, dfeed);

    // make a list of input signals to the block containing the nested simulation(s)
    blocks_inlist = inlist;

    // add an additional input signal
    blocks_inlist($+1) = NitSignal; 

    // connect all inputs
    [sim,blk] = libdyn_conn_equation(sim, blk, blocks_inlist );

    // connect all outputs and put them into the outlist-list
    outlist = list();
    if Noutp ~= 0 then
        for i = 0:(Noutp-1)
            [sim,out] = libdyn_new_oport_hint(sim, blk, i);   // ith port
            outlist(i+1) = out;
        end
    else
        null;
        //      printf("ld_simnest: No outputs to connect\n");
    end

    // connect additional outputs
    //   [sim,computation_finished] = libdyn_new_oport_hint(sim, blk, Noutp+0);   // the last port


endfunction













function [sim, outlist, computation_finished] = ld_simnest(sim, ev, inlist, insizes, outsizes, intypes, outtypes, fn_list, dfeed, asynchron_simsteps, switch_signal, reset_trigger_signal  ) // PARSEDOCU_BLOCK
    // 
    // %PURPOSE: create one (or multiple) nested libdyn simulation within a normal libdyn block it is possible to switch between them by an special input signal
    //
    // INPUTS: 
    //
    // switch_signal: signal used to switch between different nested simulations
    // reset_trigger_signal: when 1 the current simulation is reset (sync)
    //                       OR when 1 the current simulation is triggered (async)
    // inlist - list( ) of input signals to the block, that will be forwarded to the nested simulation(s)
    //
    // PARAMETERS:
    // 
    // ev - events to be forwarded to the nested simulation
    // insizes - input ports configuration
    // outsizes - output ports configuration
    // intypes - ignored for now, put ORTD.DATATYPE_FLOAT for each port
    // outtypes - ignored for now, put ORTD.DATATYPE_FLOAT for each port
    // fn_list - list( ) of scilab functions defining sub-schematics
    // dfeed - the block containing all sub-schematics is configured with dfeed
    // asynchron_simsteps - if == 1 one simulation steps will be simulated in a thread
    //                     when finished the result becomes available to the blocks outports
    //                      if == 2 the simulation will be simulated in a thread and can be synchronised
    //                      by the subsimulation itselft through synchronisation blocks (e.g. ld_synctimer)
    //                     if == 0 the nested simulation runns synchronous to the upper level simulation. 
    //                     (i.e. no thread is started)
    // 
    // OUTPUTS:
    // 
    // outlist - list( ) of output signals
    // computation_finished - optional and only meanful if asynchron_simsteps > 0 (means async computation)
    // 


    parlist = new_irparam_set();

    N1 = length(insizes);
    N2 = length(outsizes);
    N3 = length(intypes);
    N4 = length(outtypes);

    // check input signals for correctness
    try 
        libdyn_check_object(sim, switch_signal);
    catch
        error("Input signal switch_signal is not correct");
    end

    try
        libdyn_check_object(sim, reset_trigger_signal);
    catch
        error("Input signal reset_trigger_signal is not correct");
    end


    // check for sizes
    // ...
    if (length(inlist) ~= N1) then
        error("length inlist invalid or length of insizes invalid\n");
    end

    if N4 ~= N2 then
        error("length of outsizes invalid\n");
    end

    if N1 ~= N3 then
        error("length of intypes invalid\n");
    end


    parlist = new_irparam_elemet_ivec(parlist, insizes, 10); 
    parlist = new_irparam_elemet_ivec(parlist, outsizes, 11); 
    parlist = new_irparam_elemet_ivec(parlist, intypes, 12); 
    parlist = new_irparam_elemet_ivec(parlist, outtypes, 13); 


    if (length(dfeed) ~= 1) then
        error("dfeed should be of size 1\n");
    end

    if (length(asynchron_simsteps) ~= 1) then
        error("asynchron_simsteps should be of size 1\n");
    end


    Nsimulations = length(fn_list);
    parlist = new_irparam_elemet_ivec(parlist, [Nsimulations, dfeed, asynchron_simsteps], 20); 



    // Go through all schematics
    // and set them up
    // finially they are stored within an irpar structure under different irpar ids
    irpar_sim_idcounter = 900;

    for i = 1:Nsimulations

        fn = fn_list(i);
        //    [sim_container_irpar, sim] = libdyn_setup_schematic(fn, insizes, outsizes, intypes, outtypes); // outtypes and intypes are not handled at the moment
        [sim_container_irpar, nested_sim] = libdyn_setup_schematic(fn, insizes, outsizes); 

        // pack simulations into irpar container with id = 901
        parlist = new_irparam_container(parlist, sim_container_irpar, irpar_sim_idcounter);

        // increase irpar_sim_idcounter so the next simulation gets another id
        irpar_sim_idcounter = irpar_sim_idcounter + 1;
    end



    //   // combine ir parameters
    blockparam = combine_irparam(parlist);


    // blockparam.ipar and blockparam.rpar now contain the blocks parameters

    // set-up the nested block

    btype = 15001;
    //   [sim,blk] = libdyn_new_blk_generic(sim, ev, btype, [blockparam.ipar], [blockparam.rpar] );
    [sim,blk] = libdyn_new_block(sim, ev, btype, ipar=[blockparam.ipar], rpar=[blockparam.rpar], ...
    insizes=[insizes(:)' ,1,1], outsizes=[outsizes(:)', 1], ...
    intypes=[ones(insizes(:)') * ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_FLOAT], ...
    outtypes=[ones(outsizes(:)') * ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_FLOAT]  );


    // 
    // add switch and reset input signals
    blocks_inlist = inlist;
    blocks_inlist($+1) = switch_signal;

    if asynchron_simsteps == 0 then
        // connect reset input
        blocks_inlist($+1) = reset_trigger_signal; //reset_signal;
    else
        // connect trigger input
        blocks_inlist($+1) = reset_trigger_signal; //trigger_signal;
    end

    // connect all inputs
    //    printf("ld_simnest: connecting inputs\n");
    try
        [sim,blk] = libdyn_conn_equation(sim, blk, blocks_inlist );
    catch
        error("ld_simnest: One of the input signals in inlist could not be connected");
    end

    // connect all outputs
    //    printf("ld_simnest: connecting outputs\n");
    Noutp = length(outsizes);

    outlist = list();

    if Noutp ~= 0 then
        for i = 0:(Noutp-1)
            [sim,out] = libdyn_new_oport_hint(sim, blk, i);   // ith port
            outlist(i+1) = out;
        end
    else
        null;
        //      printf("ld_simnest: No outputs to connect\n");
    end

    if (asynchron_simsteps > 0) then
        [sim,computation_finished] = libdyn_new_oport_hint(sim, blk, Noutp);   // the last port
    else
        computation_finished = 0; // dummy value
    end

endfunction




function [sim, outlist, computation_finished, userdata] = ld_simnest2(sim, ev, inlist, insizes, outsizes, intypes, outtypes, nested_fn, Nsimulations, dfeed, asynchron_simsteps, switch_signal, reset_trigger_signal, userdata, simnest_name)  // PARSEDOCU_BLOCK
    // 
    // %PURPOSE: create one (or multiple) nested libdyn simulation within a normal libdyn block
    //                It is possible to switch between them by an special input signal
    //                Replacement schematics can be generated by the "ld_simnest2_replacement" function.
    //                The "ld_nested_exchffile"-block can be used to load the schematic into the controller
    //
    // INPUTS: 
    //
    // switch_signal: signal used to switch between different nested simulations
    // reset_trigger_signal: when 1 the current simulation is reset (sync)
    //                       OR when 1 the current simulation is triggered (async)
    // inlist - list( ) of input signals to the block, that will be forwarded to the nested simulation(s)
    //
    // PARAMETERS:
    // 
    // ev - events to be forwarded to the nested simulation
    // insizes - input ports configuration
    // outsizes - output ports configuration
    // intypes - ignored for now, put ORTD.DATATYPE_FLOAT for each port
    // outtypes - ignored for now, put ORTD.DATATYPE_FLOAT for each port
    // nested_fn - scilab function defining sub-schematics
    // Nsimulations - number of simulations to switch inbetween
    // dfeed - the block containing all sub-schematics is configured with dfeed
    // asynchron_simsteps - if > 0 asynchron_simsteps steps will be simulated in a thread
    //                     when finished the result becomes available to the blocks outports
    //                     if == 0 the nested simulation runns synchronous to the upper level simulation.
    // switch_signal 
    // reset_trigger_signal
    // userdata - arbitrary user date
    // simnest_name - a string name with which the nested is refered.
    // 
    // OUTPUTS:
    // 
    // outlist - list( ) of output signals
    // computation_finished - optional and only meanful if asynchron_simsteps > 0 (means async computation)
    // 


    parlist = new_irparam_set();

    N1 = length(insizes);
    N2 = length(outsizes);
    N3 = length(intypes);
    N4 = length(outtypes);

    // check for sizes
    // ...
    if (length(inlist) ~= N1) then
        error("length inlist invalid or length of insizes invalid\n");
    end

    if N4 ~= N2 then
        error("length of outsizes invalid\n");
    end

    if N1 ~= N3 then
        error("length of intypes invalid\n");
    end


    parlist = new_irparam_elemet_ivec(parlist, insizes, 10); 
    parlist = new_irparam_elemet_ivec(parlist, outsizes, 11); 
    parlist = new_irparam_elemet_ivec(parlist, intypes, 12); 
    parlist = new_irparam_elemet_ivec(parlist, outtypes, 13); 


    if (length(dfeed) ~= 1) then
        error("dfeed should be of size 1\n");
    end

    if (length(asynchron_simsteps) ~= 1) then
        error("asynchron_simsteps should be of size 1\n");
    end



    parlist = new_irparam_elemet_ivec(parlist, [Nsimulations, dfeed, asynchron_simsteps], 20); 
    parlist = new_irparam_elemet_ivec(parlist, ascii(simnest_name), 21); 

    //    
    // FIXME: USE THIS FOR A NEW FN ld_async_simulation
    //    

    //    // varargin
    //    rhs=argn(2);
    //    if ( rhs > 15 ) then
    //    
    //      if asynchron_simsteps ~= 0 then     
    //        printf("Adding optional thread priority information\n");     
    //        ThreadPrioStruct = varargin(1);
    //        
    //       // also add thread priority
    //       try
    //         ThreadPrio = [ThreadPrioStruct.prio1, ThreadPrioStruct.prio2, ThreadPrioStruct.cpu];
    //       catch
    //         error("Structure for thread priority no ok. Must be ThreadPrioStruct.prio1, ThreadPrioStruct.prio2, ThreadPrioStruct.cpu");
    //       end
    //       parlist = new_irparam_elemet_ivec(parlist, ThreadPrio, 22); 
    //     end
    //     
    //   end

    // Go through all schematics
    // and set them up
    // finially they are stored within an irpar structure under different irpar ids
    irpar_sim_idcounter = 900;

    for i = 1:Nsimulations

        //    [sim_container_irpar, sim] = libdyn_setup_schematic(fn, insizes, outsizes, intypes, outtypes); // outtypes and intypes are not handled at the moment
        //[sim_container_irpar, nested_sim] = libdyn_setup_schematic(fn, insizes, outsizes); 
        [sim_container_irpar, nested_sim, userdata] = libdyn_setup_sch2(nested_fn, insizes, outsizes,  intypes, outtypes, list(i, userdata)); 

        // pack simulations into irpar container with id = 901
        parlist = new_irparam_container(parlist, sim_container_irpar, irpar_sim_idcounter);

        // increase irpar_sim_idcounter so the next simulation gets another id
        irpar_sim_idcounter = irpar_sim_idcounter + 1;
    end



    //   // combine ir parameters
    blockparam = combine_irparam(parlist);


    // blockparam.ipar and blockparam.rpar now contain the blocks parameters

    // set-up the nested block

    btype = 15001;
    //   [sim,blk] = libdyn_new_blk_generic(sim, ev, btype, [blockparam.ipar], [blockparam.rpar] );
    [sim,blk] = libdyn_new_block(sim, ev, btype, ipar=[blockparam.ipar], rpar=[blockparam.rpar], ...
    insizes=[insizes(:)' ,1,1], outsizes=[outsizes(:)', 1], ...
    intypes=[ones(insizes(:)') * ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_FLOAT], ...
    outtypes=[ones(outsizes(:)') * ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_FLOAT]  );


    // 
    // add switch and reset input signals
    blocks_inlist = inlist;
    blocks_inlist($+1) = switch_signal;

    if asynchron_simsteps == 0 then
        // connect reset input
        blocks_inlist($+1) = reset_trigger_signal; //reset_signal;
    else
        // connect trigger input
        blocks_inlist($+1) = reset_trigger_signal; //trigger_signal;
    end

    // connect all inputs
    [sim,blk] = libdyn_conn_equation(sim, blk, blocks_inlist );

    // connect all outputs
    Noutp = length(outsizes);

    outlist = list();
    if Noutp ~= 0 then
        for i = 0:(Noutp-1)
            [sim,out] = libdyn_new_oport_hint(sim, blk, i);   // ith port
            outlist(i+1) = out;
        end
    else
        null;
        //      printf("ld_simnest: No outputs to connect\n");
    end

    if (asynchron_simsteps > 0) then
        [sim,computation_finished] = libdyn_new_oport_hint(sim, blk, Noutp);   // the last port
    else
        computation_finished = 0; // dummy value
    end

endfunction


//   // FIXME: change PARSEDOCU_BLOCK to   // PARSEDOCU_FUNCTION
function [par, userdata] = ld_simnest2_replacement( insizes, outsizes, intypes, outtypes, nested_fn, userdata, N  )   // PARSEDOCU_BLOCK
    // 
    // %PURPOSE: create schematics that can be used as an online exchangeable simulation for nested simulations set-up using the ld_nested2 block
    //
    // The "ld_nested_exchffile"-block can be used to load the schematic into the controller
    //
    // INPUTS: 
    //
    // switch_signal: signal used to switch between different nested simulations
    // reset_trigger_signal: when 1 the current simulation is reset (sync)
    //                       OR when 1 the current simulation is triggered (async)
    // inlist - list( ) of input signals to the block, that will be forwarded to the nested simulation(s)
    //
    // PARAMETERS:
    // 
    // insizes - input ports configuration
    // outsizes - output ports configuration
    // intypes - ignored for now, put ORTD.DATATYPE_FLOAT for each port
    // outtypes - ignored for now, put ORTD.DATATYPE_FLOAT for each port
    // nested_fn - scilab function defining sub-schematics
    // N - slot id; set to 2
    // 
    // OUTPUTS:
    //
    // par - irpar data set. par.ipar and par.rpar contain the integer and double parameters list
    // 


    parlist = new_irparam_set();

    N1 = length(insizes);
    N2 = length(outsizes);
    N3 = length(intypes);
    N4 = length(outtypes);

    // check for sizes
    // ...

    if N4 ~= N2 then
        error("length of outsizes invalid\n");
    end

    if N1 ~= N3 then
        error("length of intypes invalid\n");
    end

    if length(N) ~= 1 then
        error("N must be scalar\n");
    end

    parlist = new_irparam_elemet_ivec(parlist, insizes, 10); 
    parlist = new_irparam_elemet_ivec(parlist, outsizes, 11); 
    parlist = new_irparam_elemet_ivec(parlist, intypes, 12); 
    parlist = new_irparam_elemet_ivec(parlist, outtypes, 13); 



    parlist = new_irparam_elemet_ivec(parlist, [ -1, -1, -1, getdate("s"), -1, -1, getdate() ], 21); 
    parlist = new_irparam_elemet_ivec(parlist, [ ascii("ORTD-replacement-schematic") ], 22); 



    // Go through all schematics
    // and set them up
    // finially they are stored within an irpar structure under different irpar ids
    irpar_sim_idcounter = 100;

    for i = N

        //    [sim_container_irpar, sim] = libdyn_setup_schematic(fn, insizes, outsizes, intypes, outtypes); // outtypes and intypes are not handled at the moment
        //[sim_container_irpar, nested_sim] = libdyn_setup_schematic(fn, insizes, outsizes); 
        //  printf("***\n");
        //  pause;
        [sim_container_irpar, nested_sim, userdata] = libdyn_setup_sch2(nested_fn, insizes, outsizes, intypes, outtypes, list(i, userdata) ); 

        // pack simulations into irpar container with id = 901
        parlist = new_irparam_container(parlist, sim_container_irpar, irpar_sim_idcounter);

        // increase irpar_sim_idcounter so the next simulation gets another id
        irpar_sim_idcounter = irpar_sim_idcounter + 1;
    end



    //   // combine ir parameters
    par = combine_irparam(parlist);
    // blockparam.ipar and blockparam.rpar now contain the blocks parameters

    //   ipar = blockparam.ipar;
    //   rpar = blockparam.rpar;

endfunction





function [sim, outlist, x_global, active_state, userdata] = ld_statemachine(sim, ev, inlist, insizes, outsizes, intypes, outtypes, nested_fn, Nstates, state_names_list, inittial_state, x0_global, userdata  ) // PARSEDOCU_BLOCK
    // 
    // %PURPOSE: A statemachine

    //  create one (or multiple) nested libdyn simulation within a normal libdyn block
    //                    it is possible to switch between them by an special signal. By this the
    //                    nested simulations can be interpreted as a state machine, whereby each
    //                    simulation describes a state. Additionally, global states (a vector of doubles)
    //                    can be shared accross the different nested simulations.
    //
    // EXAMPLE: examples/state_machine.sce (probpably tells more then this reference)
    //
    // INPUTS: 
    //
    // inlist - list( ) of input signals to the block, that will be forwarded to the nested simulation(s)
    //
    // PARAMETERS:
    // 
    // ev - events to be forwarded to the nested simulation. The first one is also used by this nesting block as activation event
    // insizes - input ports configuration
    // outsizes - output ports configuration
    // intypes - ignored for now, put ORTD.DATATYPE_FLOAT for each port
    // outtypes - ignored for now, put ORTD.DATATYPE_FLOAT for each port
    // nested_fn - list( ) of one! scilab functions defining sub-schematics
    // Nstates - number of simulations to switch inbetween
    // state_names_list - list() of strings - one for each state
    // inittial_state - number of the inittial state. counting starts at 1
    // x0_global - inittial state of x_global
    // userdata - anythig - feed to the schematic_fn
    // 
    // OUTPUTS:
    // 
    // outlist - list( ) of output signals
    // x_global - signal of the states x_global
    // active_state - signal with the number of the currently active state
    // 
    // PROTOTYPE FOR nested_fn:
    // 
    // [sim, outlist, active_state, x_global_kp1, userdata] = state_mainfn(sim, inlist, x_global, state, statename, userdata)
    //
    // inlist - the same signals as feed to ld_statemachine
    // x_global - vector signal of the global states
    // state - constant: number of the state to define
    // statename - string: name of the state to define
    // userdata - a custom variable as feed to ld_statemachine
    //
    // RETURS:
    //
    // outlist - 
    // active_state - signal describing the active state. If this is a singal containing -1 no state swicth occurs
    // x_global_kp1 - vectorial signal of the new global states 
    //
    // PLEASE NOTE: LIMITATIONS:
    // 
    // only ORTD.DATATYPE_FLOAT is supported!
    // 
    // A fixed "ld_statemachine2" is comming, makeing this implementation obsolete.
    // 





    function [sim, outlist, userdata ] = LD_STATEMACHINE_MAIN(sim, inlist, userdata)
        // wrapper function

        state = userdata(1);
        userdata_nested = userdata(2);
        fn = userdata(3);
        statename = userdata(4);  
        Nin_userdata = userdata(5);  
        Nout_userdata = userdata(6); 
        Nevents = userdata(7);
        //  Nevents = 1;


        printf("ld_statemachine: defining state=%s (#%d)\n", statename, state);
        //     pause;

        x_global = inlist(Nin_userdata+1); // the global states

        // rmeove the x_global signal form the inlist_inner. Added, Bugfix 5.8.14
        inlist = list( inlist(1:Nin_userdata) );

        // pause;

        // call the actual function
        [sim, outlist_inner, active_state, x_global_kp1, userdata_nested] = fn(sim, inlist, x_global, state, statename, userdata_nested);

        // pause;

        if length(outlist_inner) ~= Nout_userdata then
            printf("ld_statemachine: your provided schmatic-describing function returns more or less outputs in outlist. Expecting %d but there are %d\n", Nout_userdata, length(outlist_inner));
            error(".");
        end

        outlist = list();

        N = length(outlist_inner);
        for i = 1:N
            outlist($+1) = outlist_inner(i);
        end
        outlist($+1) = active_state;
        outlist($+1) = x_global_kp1

        userdata = userdata_nested;

        // pause;
    endfunction



    parlist = new_irparam_set();

    N1 = length(insizes);
    N2 = length(outsizes);
    N3 = length(intypes);
    N4 = length(outtypes);

    NGlobStates = length(x0_global);
    N_datainports = length(insizes);
    N_dataoutports = length(outsizes);


    // check for sizes
    // ...
    if (length(inlist) ~= N1) then
        error("length inlist invalid or length of insizes invalid\n");
    end

    if N4 ~= N2 then
        error("length of outsizes invalid\n");
    end

    if N1 ~= N3 then
        error("length of intypes invalid\n");
    end


    // io for the nested simulation (this is not eq to the outer blocks io)

    nested_insizes = [ insizes(:)', NGlobStates ];
    nested_outsizes = [ outsizes(:)', 1, NGlobStates];
    nested_intypes = [ intypes(:)', ORTD.DATATYPE_FLOAT];
    nested_outtypes = [ outtypes(:)', ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_FLOAT ];

    parlist = new_irparam_elemet_ivec(parlist, nested_insizes, 10); 
    parlist = new_irparam_elemet_ivec(parlist, nested_outsizes, 11);  // datasigs, active_state, x_global
    parlist = new_irparam_elemet_ivec(parlist, nested_intypes, 12); 
    parlist = new_irparam_elemet_ivec(parlist, nested_outtypes, 13); 



    // parameters
    parlist = new_irparam_elemet_ivec(parlist, [Nstates, NGlobStates, inittial_state], 20); 
    parlist = new_irparam_elemet_rvec(parlist, [x0_global], 21);  // inittial global state



    // Go through all schematics
    // and set them up
    // finially they are stored within an irpar structure under different irpar ids
    irpar_sim_idcounter = 900;

    for i = 1:Nstates
        // the parameters for the wrapper scilab function
        wrapper_fn_arg = list(i, userdata, nested_fn, state_names_list(i), N_datainports, N_dataoutports, length(ev) );

        // create a nested simulation
        [sim_container_irpar, nested_sim, userdata] = libdyn_setup_sch2(LD_STATEMACHINE_MAIN, insizes=nested_insizes, ...
        outsizes=nested_outsizes, nested_intypes, nested_outtypes, wrapper_fn_arg); 

        // pack simulations into irpar container with id = 901
        parlist = new_irparam_container(parlist, sim_container_irpar, irpar_sim_idcounter);

        // increase irpar_sim_idcounter so the next simulation gets another id
        irpar_sim_idcounter = irpar_sim_idcounter + 1;
    end



    //   // combine ir parameters
    blockparam = combine_irparam(parlist);


    // blockparam.ipar and blockparam.rpar now contain the blocks parameters

    // set-up the nested block



    btype = 15002;
    //   [sim,blk] = libdyn_new_blk_generic(sim, ev, btype, [blockparam.ipar], [blockparam.rpar] );
    [sim,blk] = libdyn_new_block(sim, ev, btype, ipar=[blockparam.ipar], rpar=[blockparam.rpar], ...
    insizes=[insizes(:)'], outsizes=[outsizes(:)', 1, NGlobStates], ...
    intypes=[intypes(:)'], ...
    outtypes=[outtypes(:)', ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_FLOAT]  );


    // 
    // add switch and reset input signals
    blocks_inlist = inlist;

    // connect all inputs
    [sim,blk] = libdyn_conn_equation(sim, blk, blocks_inlist );

    // connect all outputs
    Noutp = length(outsizes);

    outlist = list();
    for i = 0:(Noutp-1)
        [sim,out] = libdyn_new_oport_hint(sim, blk, i);   // ith port
        outlist(i+1) = out;
    end

    [sim,active_state] = libdyn_new_oport_hint(sim, blk, Noutp);   // the second last port
    [sim,x_global] = libdyn_new_oport_hint(sim, blk, Noutp+1);   // the last port

endfunction



function [sim, out] = ld_nested_exchffile(sim, events, compresult, slot, fname, simnest_name) // PARSEDOCU_BLOCK
    //
    // %PURPOSE: Online exchange of a nested simulation via loading *[ir].par files
    //
    // NOTE: May delay realtime behaviour - use during a separated asynchronous nested simulation
    //
    // compresult - signal - 0: Do nothing, 1: unload schematic and load new (replace), 2: just unload 
    // slot - signal (unused by now; set to 2)
    // fname - the filename without ".[ir]par"-ending
    // simnest_name - the string which referes to the nested simulation (as given to "ld_simnest2")
    // 
    //

    ifname = fname + ".ipar";
    rfname = fname + ".rpar";

    btype = 15003;
    [sim,blk] = libdyn_new_block(sim, events, btype, [0,0,0, length(ifname), length(rfname), length(simnest_name), ascii(ifname), ascii(rfname), ascii(simnest_name) ], [ ], ...
    insizes=[1, 1], outsizes=[1], ...
    intypes=[ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_FLOAT], ...
    outtypes=[ORTD.DATATYPE_FLOAT]  );

    [sim,blk] = libdyn_conn_equation(sim, blk, list(compresult, slot) );
    [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction


function [sim, outvec] = ld_survivereset(sim, events, invec, initvec) // PARSEDOCU_BLOCK
    // %PURPOSE: Keep stored data even if a simulation reset occurs (EXPERIMENTAL FOR NOW)
    //
    // in *+(vecsize) - input
    // out *+(vecsize) - output
    // 
    // Prior to a simulation reset the input is stored into memory.
    // After the reset the data is available again via the output
    // Initially the output is set to initvec
    // 
    //    

    if (length(initvec) < 1) then
        error("length(initvec) < 1 !");
    end

    btype = 15004;	
    ipar = [length(initvec); 0]; rpar = [initvec];

    [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
    insizes=[length(initvec)], outsizes=[length(initvec)], ...
    intypes=[ ORTD.DATATYPE_FLOAT ], outtypes=[ORTD.DATATYPE_FLOAT] );

    // libdyn_conn_equation connects multiple input signals to blocks
    [sim,blk] = libdyn_conn_equation(sim, blk, list( in ) );

    [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction







// 
// A higher level for defining online replaceable schematics using callback functions
// 


function [sim, outlist, userdata, replaced] = ld_ReplaceableNest(sim, ev, inlist, trigger_reload, fnlist, insizes, outsizes, intypes, outtypes, simnest_name, irpar_fname, dfeed, userdata) // PARSEDOCU_BLOCK
    // %PURPOSE: A higher level for defining online replaceable schematics using callback functions
    //
    // inlist - input list()
    // outlist - output list()
    // 
    // For an example see modules/nested/demo/online_replacement
    //    


    function [sim, outlist] = exch_helper_thread(sim, inlist)
        // This superblock will run the evaluation of the experiment in a thread.
        // The superblock describes a sub-simulation, whereby only one step is simulated
        // which is enough to call scilab one signle time


        defaultevents = 0;

        inputv = inlist(1);

        //   [sim] = ld_printf(sim, defaultevents, inputv, "inputv = ", 10);

        //
        // A resource demanding Scilab calculation
        //

        [sim, dummyin] = ld_const(sim, defaultevents, 1);

        [sim, compready]  = ld_const(sim, defaultevents, 1);
        [sim, result] = ld_constvec(sim, defaultevents, vec=1:10)

        // replace schematic RST_ident
        [sim, exchslot] = ld_const(sim, defaultevents, 2);                            
        [sim, out] = ld_nested_exchffile(sim, defaultevents, compresult=compready, slot=exchslot, ... 
        fname=irpar_fname, simnest_name);


        // output of schematic
        outlist = list(result);
    endfunction




    function [sim, outlist, userdata ] = replaceable_cntrl_main(sim, inlist, par)
        //    
        //    The nested simulation contains two sub-simulations:
        //    
        //    1) A schematic, which commonly contains nothing and is switched to
        //       when the replacement is in progress (which may take some time)
        //
        //    2) The schematic, which actually contains the algorithm to execute
        //
        //    Here, the initial simulations are defined, which can then be 
        //   replaced during runtime
        //    


        ev = 0;

        cntrlN = par(1); // the number of the nested schematics (one of two) "1" means the 
        // dummy schematic which is activated while the 2nd "2" is exchanged during runtime
        userdata = par(2);

        useruserdata = userdata(1);
        define_fn = userdata(2);

        if (cntrlN == 1) then  // is this the schematic 2) ?
            [sim, outlist, useruserdata] = define_fn( sim, inlist, schematic_type='spare', useruserdata );
        end

        if (cntrlN == 2) then  // is this the schematic 2) ?
            [sim, outlist, useruserdata] = define_fn( sim, inlist, schematic_type='default', useruserdata );
        end

        userdata(1) = useruserdata;
    endfunction

    [sim,zero] = ld_const(sim, ev, 0);
    [sim,active_sim] = ld_const(sim, ev, 1);

    // get the callback functions
    define_fn = fnlist(1);


    //
    // Here the controller is nested, which can be replaced online
    //

    [sim, outlist_42342, computation_finished, userdata] = ld_simnest2(sim, ev=[ ev ] , ...
    inlist, ...
    insizes, outsizes, ...
    intypes, ...
    outtypes, ...
    nested_fn=replaceable_cntrl_main, Nsimulations=2, dfeed, ...
    asynchron_simsteps=0, switch_signal=active_sim, ...
    reset_trigger_signal=zero, userdata=list(userdata, define_fn), ...
    simnest_name );

    //        [sim] = ld_printf(sim, ev, in=outlist(1), str="The nested, replaceable sim returns", insize=1);


    //
    // The exchange helper, which consits of a threaded sub-simulation
    // for loading the schematic from files.
    // The replacement is triggered by setting "startcalc" to 1 for one sample
    //

    // input to the calculation
    [sim, input1] = ld_constvec(sim, ev, vec=1:10);

    [sim, zero] = ld_const(sim, ev, 0);
    //        [sim, startcalc] = ld_initimpuls(sim, ev);
    startcalc = trigger_reload;


    // a nested simulation that runns asynchronously (in a thread) to the main simulation
    [sim, outlist__, computation_finished] = ld_simnest(sim, ev, ...
    inlist=list(input1), ...
    insizes=[10], outsizes=[10], ...
    intypes=[ORTD.DATATYPE_FLOAT], outtypes=[ORTD.DATATYPE_FLOAT], ...
    fn_list=list(exch_helper_thread), ...
    dfeed=1, asynchron_simsteps=1, ...
    switch_signal=zero, reset_trigger_signal=startcalc         );

    output1 = outlist__(1);
    // computation_finished is one, when finished else zero

    replaced = computation_finished; // FIXME and successful

    // The output list() of the nested schematic
    outlist = outlist_42342;


endfunction








function [sim] = ld_global_memory(sim, events, ident_str, datatype, len, initial_data, visibility, useMutex)  // PARSEDOCU_BLOCK
    // 
    // %PURPOSE: inittialise a persistent globally shared memory
    // 
    // ident_str (string) - name of the memory
    // datatype - ORTD datatype of the memory (for now only ORTD.DATATYPE_FLOAT works)
    // len (integer) - number of elements
    // initial_data - initial data of the memory
    // visibility (string) - 'global', ... (more are following)
    // useMutex (integer) - 0 or 1. Use a mutex if you access the memory from different threads
    // 
    // 

    // check parameters
    ortd_checkpar(sim, list('String', 'ident_str', ident_str) );
    ortd_checkpar(sim, list('SingleValue', 'datatype', datatype) );  // FIXME: Change this to 'SingleORTDDatatype'
    ortd_checkpar(sim, list('SingleValue', 'len', len) );
    ortd_checkpar(sim, list('Vector', 'initial_data', initial_data) );
    ortd_checkpar(sim, list('String', 'visibility', visibility) );
    ortd_checkpar(sim, list('SingleValue', 'useMutex', useMutex) );



    ident_str = ident_str + '.memory';

    if (visibility == 'global') then

    else
        error("Visibility has to be one of global, ... (more are following)");
    end

    btype = 15001 + 4   ;	
    ipar = [0, datatype, len, useMutex, 0,0,0,0, 0,0, length(ident_str), ascii(ident_str) ]; 

    if datatype == ORTD.DATATYPE_FLOAT then
        rpar = [ initial_data ];
        if (length(initial_data) ~= len) then
            error("length(initial_data) ~= len");
        end
    else
        rpar = [ ];
        error("datatype is not one of ORTD.DATATYPE_FLOAT, ...");
    end

    [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
    insizes=[], outsizes=[], ...
    intypes=[], outtypes=[]  );

    // ensure the block is included in the simulation even without any I/O ports
    sim = libdyn_include_block(sim, blk);

    //   [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

function [sim] = ld_write_global_memory(sim, events, data, index, ident_str, datatype, ElementsToWrite)   // PARSEDOCU_BLOCK
    // 
    // %PURPOSE: Write a portion to a persistent globally shared memory
    // 
    // Initialises a memory structure which can be refered by an
    // identifier. Data is available for read and write access
    // accross different state machines as well as accross
    // different threads.
    // 
    // Make sure to only use the memory created by this function in 
    // lower level simulations such as nested state machines, etc.
    // Access from higher level simulations is possible but should
    // be avoided, as the memory can not be freed on destruction.
    // 
    // data *+(ElementsToWrite) - data
    // index * - index to store the data. Starts at 1
    // ident_str (string) - name of the memory
    // datatype - ORTD datatype of the memory (for now only ORTD.DATATYPE_FLOAT)
    // ElementsToWrite (integer) - number of elements to write to the memory
    // 
    // 


    // check parameters
    ortd_checkpar(sim, list('Signal', 'data', data) );
    ortd_checkpar(sim, list('Signal', 'index', index) );
    ortd_checkpar(sim, list('String', 'ident_str', ident_str) );
    ortd_checkpar(sim, list('SingleValue', 'datatype', datatype) );  // FIXME: Change this to 'SingleORTDDatatype'
    ortd_checkpar(sim, list('SingleValue', 'ElementsToWrite', ElementsToWrite) );


    ident_str = ident_str + '.memory';

    btype = 15001 + 5   ;	
    ipar = [0, datatype, ElementsToWrite, 0, 0,0,0,0, 0,0, length(ident_str), ascii(ident_str) ]; 
    rpar = [];

    [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
    insizes=[ElementsToWrite , 1], outsizes=[], ...
    intypes=[ datatype, ORTD.DATATYPE_FLOAT  ], outtypes=[]  );

    // ensure the block is included in the simulation even without any I/O ports
    [sim,blk] = libdyn_conn_equation(sim, blk, list(data, index) );



    //   [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

function [sim, data] = ld_read_global_memory(sim, events, index, ident_str, datatype, ElementsToRead)   // PARSEDOCU_BLOCK
    // 
    // %PURPOSE: Read a portion from a persistent globally shared memory
    // 
    // data *+(ElementsToRead) - data read from memory
    // index * - start-index to read the data. Starts at 1
    // ident_str (string) - name of the memory
    // datatype - ORTD datatype of the memory (for now only ORTD.DATATYPE_FLOAT)
    // ElementsToRead (integer) - number of elements to read from the memory
    // 

    // check parameters
    ortd_checkpar(sim, list('Signal', 'index', index) );
    ortd_checkpar(sim, list('String', 'ident_str', ident_str) );
    ortd_checkpar(sim, list('SingleValue', 'datatype', datatype) );  // FIXME: Change this to 'SingleORTDDatatype'
    ortd_checkpar(sim, list('SingleValue', 'ElementsToRead', ElementsToRead) );


    ident_str = ident_str + '.memory';

    btype = 15001 + 6;
    ipar = [0, datatype, ElementsToRead, 0, 0,0,0,0, 0,0, length(ident_str), ascii(ident_str) ]; 
    rpar = [];

    [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
    insizes=[1], outsizes=[ElementsToRead], ...
    intypes=[ ORTD.DATATYPE_FLOAT ], outtypes=[datatype*[1]]  );

    // ensure the block is included in the simulation even without any I/O ports
    [sim,blk] = libdyn_conn_equation(sim, blk, list(index) );

    [sim,data] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

function [sim] = ld_WriteMemory2(sim, events, data, index, ElementsToWrite, ident_str, datatype, MaxElements)   // PARSEDOCU_BLOCK
    // 
    // %PURPOSE: Write a portion to a persistent globally shared memory
    // 
    // Initialises a memory structure which can be refered by an
    // identifier. Data is available for read and write access
    // accross different state machines as well as accross
    // different threads.
    // 
    // Make sure to only use the memory created by this function in 
    // lower level simulations such as nested state machines, etc.
    // Access from higher level simulations is possible but should
    // be avoided, as the memory can not be freed on destruction.
    // 
    // data *+(MaxElements) - data
    // index * INT32 - index to store the data. Starts at 1
    // ElementsToWrite * INT32 - number of elements to write to the memory
    // ident_str (string) - name of the memory
    // datatype - ORTD datatype of the memory (for now only ORTD.DATATYPE_FLOAT)
    // MaxElements - maximal elements to write
    // 
    // 


    // check parameters
    ortd_checkpar(sim, list('Signal', 'data', data) );
    ortd_checkpar(sim, list('Signal', 'index', index) );
    ortd_checkpar(sim, list('Signal', 'ElementsToWrite', ElementsToWrite) );
    ortd_checkpar(sim, list('String', 'ident_str', ident_str) );
    ortd_checkpar(sim, list('SingleValue', 'datatype', datatype) );  // FIXME: Change this to 'SingleORTDDatatype'
    ortd_checkpar(sim, list('MaxElements', 'datatype', datatype) );


    ident_str = ident_str + '.memory';

    btype = 15001 + 7   ;	
    ipar = [0, datatype, MaxElements, 0, 0,0,0,0, 0,0, length(ident_str), ascii(ident_str) ]; 
    rpar = [];

    [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
    insizes=[MaxElements , 1, 1], outsizes=[], ...
    intypes=[ datatype, ORTD.DATATYPE_INT32, ORTD.DATATYPE_INT32  ], outtypes=[]  );


    [sim,blk] = libdyn_conn_equation(sim, blk, list(data, index, ElementsToWrite) );
endfunction




// 
// 
// Interfacing functions are placed in this place
// 
// 
// This is a template from which scilab_loader.sce is automatically produced
// when running the module's makefile.
//
// The placeholder 999911111 will be repalced when running the Makefile by the 
// contents of the variable blockid_start in the beginning of the Makefile
// 


// 
// ortd_checkpar types:
// 
//     'Signal' 
//     'SignalList' 
//     'SingleValue' 
//     'Vector'
//     'String'
// 
//  e.g.
// 
//   ortd_checkpar(sim, list('Signal', 'in', in) );
//   ortd_checkpar(sim, list('SingleValue', 'gain', gain) );
// 




function [sim, outlist, Nsamples] = ld_TMSI(sim, events, DeviceIDStr, channels, MaxBlocksize ) // PARSEDOCU_BLOCK
// ADD SOME DOCUMENTATION HERE, that will be copied to the scilab help
// TMSI - block
//
// This block syncronises schematics. Only one block of this type is allowed in a threaded sub-schematic
// created e.g. by ld_async_simulation
//
// in * - input
// out * - output
// 
// out = abs(in)
// 

   // check the input parameters
   ortd_checkpar(sim, list('String', 'str', DeviceIDStr) );
   ortd_checkpar(sim, list('SingleValue', 'MaxBlocksize', MaxBlocksize) );


// introduce some parameters that are refered to by id's
Nch = length(channels);

   // pack all parameters into a structure "parlist"
   parlist = new_irparam_set();

   parlist = new_irparam_elemet_ivec(parlist, MaxBlocksize, 10); // id = 10
   parlist = new_irparam_elemet_ivec(parlist, channels, 11); // vector of integers (double vectors are similar, replace ivec with rvec)
   parlist = new_irparam_elemet_ivec(parlist, ascii(DeviceIDStr), 12); // id = 12; A string parameter

   p = combine_irparam(parlist); // convert to two vectors of integers and floating point values respectively

// Set-up the block parameters and I/O ports
  Uipar = [ p.ipar ];
  Urpar = [ p.rpar ];
  btype = 999911111 + 0; // Reference to the block's type (computational function). Use the same id you are giving via the "libdyn_compfnlist_add" C-function

  insizes=[1]; // Input port sizes
  outsizes=[ 1; MaxBlocksize*ones(Nch,1) ]; // Output port sizes
  dfeed=[1];  // for each output 0 (no df) or 1 (a direct feedthrough to one of the inputs)
  intypes=[ORTD.DATATYPE_INT32 ]; // datatype for each input port
  outtypes=[ORTD.DATATYPE_INT32; ORTD.DATATYPE_FLOAT*ones(Nch,1) ]; // datatype for each output port

  blocktype = 1; // 1-BLOCKTYPE_DYNAMIC (if block uses states), 2-BLOCKTYPE_STATIC (if there is only a static relationship between in- and output)

  // Create the block
  [sim, blk] = libdyn_CreateBlockAutoConfig(sim, events, btype, blocktype, Uipar, Urpar, insizes, outsizes, intypes, outtypes, dfeed);

  [sim, blocksize] = ld_constvecInt32(sim, 0, MaxBlocksize)

  // connect the inputs
 [sim,blk] = libdyn_conn_equation(sim, blk, list(blocksize) ); // connect in1 to port 0 and in2 to port 1

  // connect the ouputs
  
  [sim,Nsamples] = libdyn_new_oport_hint(sim, blk, 0);

  outlist = list();  
  for i=1:Nch
    [sim,out] = libdyn_new_oport_hint(sim, blk, i);   // 0th port
    outlist($+1) = out;
  end



endfunction



function [sim] = ld_ringbuf(sim, events, ident_str, datatype, len, visibility)  // PARSEDOCU_BLOCK
// 
// %PURPOSE: inittialise a new ringbuffer
// 
// ident_str (string) - name of the ringbuffer
// datatype - ORTD datatype of the ringbuffer (for now only ORTD.DATATYPE_FLOAT)
// len (integer) - number of elements
// initial_data - initial data of the ringbuffer
// visibility (string) - 'global', ... (more are following)
// 
// 

  ident_str = ident_str + '.ringbuf';

  if (visibility == 'global') then
  
  else
    error("Visibility has to be one of global, ... (more are following)");
  end

  UNUSED = 0;

  btype = 15400 + 0   ;	
  ipar = [0, datatype, len, UNUSED, 0,0,0,0, 0,0, length(ident_str), ascii(ident_str) ]; 
  
  rpar = [ ];

  if datatype == ORTD.DATATYPE_FLOAT then
//     rpar = [ initial_data ];
//     if (length(initial_data) ~= len) then
//       error("length(initial_data) ~= len");
//     end
  else
    rpar = [ ];
    error("datatype is not one of ORTD.DATATYPE_FLOAT, ...");
  end

  [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
                   insizes=[], outsizes=[], ...
                   intypes=[], outtypes=[]  );
 
 // ensure the block is included in the simulation even without any I/O ports
 sim = libdyn_include_block(sim, blk);
endfunction

function [sim] = ld_write_ringbuf(sim, events, data, ident_str, datatype, ElementsToWrite)   // PARSEDOCU_BLOCK
// 
// %PURPOSE: Write a portion to a ringbuffer
// 
// 
// data *+(ElementsToWrite) - data
// ident_str (string) - name of the memory
// datatype - ORTD datatype of the memory (for now only ORTD.DATATYPE_FLOAT)
// ElementsToWrite (integer) - number of elements to write to the memory
// 
// 

  ident_str = ident_str + '.ringbuf';

  btype = 15400 + 1   ;	
  ipar = [0, datatype, ElementsToWrite, 0, 0,0,0,0, 0,0, length(ident_str), ascii(ident_str) ]; 
  rpar = [];

  [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
                   insizes=[ElementsToWrite ], outsizes=[], ...
                   intypes=[ datatype ], outtypes=[]  );
 
 
 [sim,blk] = libdyn_conn_equation(sim, blk, list(data) );
 
 
 
//   [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction

function [sim, data, NumRead] = ld_read_ringbuf(sim, events, ident_str, datatype, ElementsToRead)   // PARSEDOCU_BLOCK
// 
// %PURPOSE: Read a portion from a ringbuffer
// 
// data *+(ElementsToRead) - data
// ident_str (string) - name of the ringbuffer
// datatype - ORTD datatype of the ringbuffer (for now only ORTD.DATATYPE_FLOAT)
// ElementsToRead (integer) - number of elements to read from the ringbuffer
// 

  ident_str = ident_str + '.ringbuf';

  btype = 15400 + 2;
  ipar = [0, datatype, ElementsToRead, 0, 0,0,0,0, 0,0, length(ident_str), ascii(ident_str) ]; 
  rpar = [];

  [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
                   insizes=[ ], outsizes=[ElementsToRead, 1], ...
                   intypes=[ ], outtypes=[datatype, ORTD.DATATYPE_FLOAT ]  );
 
 // ensure the block is included in the simulation even without any I/O ports
//  [sim,blk] = libdyn_conn_equation(sim, blk, list(index) );
 
   [sim,data] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
   [sim,NumRead] = libdyn_new_oport_hint(sim, blk, 1);   // 1th port 
   
endfunction

function [sim, data] = ld_relread_ringbuf(sim, events, ident_str, datatype, ElementsToRead, relPos)   // PARSEDOCU_BLOCK
// 
// %PURPOSE: Read a portion from a ringbuffer relative to the currently set marker position
// 
// data *+(ElementsToRead) - data
// ident_str (string) - name of the ringbuffer
// datatype - ORTD datatype of the ringbuffer (for now only ORTD.DATATYPE_FLOAT)
// ElementsToRead (integer) - number of elements to read from the ringbuffer
// relPos - UINT - position relative to the currently set marker to start reading from 
//

  ident_str = ident_str + '.ringbuf';

  btype = 15400 + 3;
  ipar = [0, datatype, ElementsToRead, 0, 0,0,0,0, 0,0, length(ident_str), ascii(ident_str) ]; 
  rpar = [];

  [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
                   insizes=[1 ], outsizes=[ElementsToRead], ...
                   intypes=[ORTD.DATATYPE_INT32 ], outtypes=[datatype ]  );
 
  // ensure the block is included in the simulation even without any I/O ports
  [sim,blk] = libdyn_conn_equation(sim, blk, list(relPos) );
 
  [sim,data] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
   
endfunction

function [sim] = ld_setmarkerW_ringbuf(sim, events, ident_str, trigger)   // PARSEDOCU_BLOCK
// 
// %PURPOSE: Set the current marker position to the lastly written element
// 
// ident_str (string) - name of the ringbuffer
// trigger - UINT - set marker if trigger > 0 
//

  ident_str = ident_str + '.ringbuf';

  btype = 15400 + 4;
  ipar = [0, 0, 0, 0, 0,0,0,0, 0,0, length(ident_str), ascii(ident_str) ]; 
  rpar = [];

  [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
                   insizes=[1 ], outsizes=[], ...
                   intypes=[ORTD.DATATYPE_INT32 ], outtypes=[ ]  );
 
  // ensure the block is included in the simulation even without any I/O ports
  [sim,blk] = libdyn_conn_equation(sim, blk, list(trigger) );
 
endfunction


// 
// 
// Interfacing functions are placed in this place
// 
// 
// This is a template from which scilab_loader.sce is automatically produced
// when running the module's makefile.
//
// The placeholder 15900 will be repalced when running the Makefile by the 
// contents of the variable blockid_start in the beginning of the Makefile
// 


// 
// ortd_checkpar types:
// 
//     'Signal' 
//     'SignalList' 
//     'SingleValue' 
//     'Vector'
//     'String'
// 
//  e.g.
// 
//   ortd_checkpar(sim, list('Signal', 'in', in) );
//   ortd_checkpar(sim, list('SingleValue', 'gain', gain) );
// 



function [sim, out] = ld_Random(sim, events, Method, Seed) // PARSEDOCU_BLOCK
// 
// Random generator - block
//
// out * - random output
// 
// Method - 0 (normal distribution), 1 (uniform distribution, NOT IMPLEMENTED)
// Seed - The random seed to start with, NOT IMPLEMENTED
//
// The implementation of http://randomlib.sourceforge.net is used.
// 

   // check the input parameters

    ortd_checkpar(sim, list('SingleValue', 'Method', Method) );
    ortd_checkpar(sim, list('SingleValue', 'Seed', Seed) );

// introduce some parameters that are refered to by id's
//    Method = 0; // normal distribution
//    Seed = 0; 

   vec = [0,Method,Seed];

   // pack all parameters into a structure "parlist"
   parlist = new_irparam_set();

   parlist = new_irparam_elemet_ivec(parlist, vec, 11); // vector of integers (double vectors are similar, replace ivec with rvec)


   p = combine_irparam(parlist); // convert to two vectors of integers and floating point values respectively

// Set-up the block parameters and I/O ports
  Uipar = [ p.ipar ];
  Urpar = [ p.rpar ];
  btype = 15900 + 0; // Reference to the block's type (computational function). Use the same id you are giving via the "libdyn_compfnlist_add" C-function

  insizes=[]; // Input port sizes
  outsizes=[1]; // Output port sizes
  dfeed=[1];  // for each output 0 (no df) or 1 (a direct feedthrough to one of the inputs)
  intypes=[]; // datatype for each input port
  outtypes=[ORTD.DATATYPE_FLOAT]; // datatype for each output port

  blocktype = 1; // 1-BLOCKTYPE_DYNAMIC (if block uses states), 2-BLOCKTYPE_STATIC (if there is only a static relationship between in- and output)

  // Create the block
  [sim, blk] = libdyn_CreateBlockAutoConfig(sim, events, btype, blocktype, Uipar, Urpar, insizes, outsizes, intypes, outtypes, dfeed);
  
  // connect the ouputs
 [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction



// 
// 
// Interfacing functions are placed in this place
// 
// 
// This is a template from which scilab_loader.sce is automatically produced
// when running the module's makefile.
//
// The placeholder 1200 will be repalced when running the Makefile by the 
// contents of the variable blockid_start in the beginning of the Makefile
// 


// 
// ortd_checkpar types:
// 
//     'Signal' 
//     'SignalList' 
//     'SingleValue' 
//     'Vector'
//     'String'
// 
//  e.g.
// 
//   ortd_checkpar(sim, list('Signal', 'in', in) );
//   ortd_checkpar(sim, list('SingleValue', 'gain', gain) );
// 



function [sim, out] = ld_TemplateWrite(sim, events, str, in1, in2) // PARSEDOCU_BLOCK
// ADD SOME DOCUMENTATION HERE, that will be copied to the scilab help
// abs - block
//
// in * - input
// out * - output
// 
// out = abs(in)
// 

   // check the input parameters
   ortd_checkpar(sim, list('String', 'str', str) );
   ortd_checkpar(sim, list('Signal', 'in1', in1) );
   ortd_checkpar(sim, list('Signal', 'in2', in2) );
//    ortd_checkpar(sim, list('SingleValue', 'gain', gain) );


// introduce some parameters that are refered to by id's
parameter1 = 12345;
vec = [1,2,3];

   // pack all parameters into a structure "parlist"
   parlist = new_irparam_set();

   parlist = new_irparam_elemet_ivec(parlist, parameter1, 10); // id = 10
   parlist = new_irparam_elemet_ivec(parlist, vec, 11); // vector of integers (double vectors are similar, replace ivec with rvec)
   parlist = new_irparam_elemet_ivec(parlist, ascii(str), 12); // id = 12; A string parameter

   p = combine_irparam(parlist); // convert to two vectors of integers and floating point values respectively

// Set-up the block parameters and I/O ports
  Uipar = [ p.ipar ];
  Urpar = [ p.rpar ];
  btype = 1200 + 0; // Reference to the block's type (computational function). Use the same id you are giving via the "libdyn_compfnlist_add" C-function

  insizes=[4,4]; // Input port sizes
  outsizes=[1]; // Output port sizes
  dfeed=[1];  // for each output 0 (no df) or 1 (a direct feedthrough to one of the inputs)
  intypes=[ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_FLOAT]; // datatype for each input port
  outtypes=[ORTD.DATATYPE_FLOAT]; // datatype for each output port

  blocktype = 1; // 1-BLOCKTYPE_DYNAMIC (if block uses states), 2-BLOCKTYPE_STATIC (if there is only a static relationship between in- and output)

  // Create the block
  [sim, blk] = libdyn_CreateBlockAutoConfig(sim, events, btype, blocktype, Uipar, Urpar, insizes, outsizes, intypes, outtypes, dfeed);
  
  // connect the inputs
 [sim,blk] = libdyn_conn_equation(sim, blk, list(in1, in2) ); // connect in1 to port 0 and in2 to port 1

  // connect the ouputs
 [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction




function [sim, out] = ld_SyncTemplate(sim, events, str, in1, in2) // PARSEDOCU_BLOCK
// ADD SOME DOCUMENTATION HERE, that will be copied to the scilab help
// abs - block
//
// This block syncronises schematics. Only one block of this type is allowed in a threaded sub-schematic
// created e.g. by ld_async_simulation
//
// in * - input
// out * - output
// 
// out = abs(in)
// 

   // check the input parameters
   ortd_checkpar(sim, list('String', 'str', str) );
   ortd_checkpar(sim, list('Signal', 'in1', in1) );
   ortd_checkpar(sim, list('Signal', 'in2', in2) );
//    ortd_checkpar(sim, list('SingleValue', 'gain', gain) );


// introduce some parameters that are refered to by id's
parameter1 = 12345;
vec = [1,2,3];

   // pack all parameters into a structure "parlist"
   parlist = new_irparam_set();

   parlist = new_irparam_elemet_ivec(parlist, parameter1, 10); // id = 10
   parlist = new_irparam_elemet_ivec(parlist, vec, 11); // vector of integers (double vectors are similar, replace ivec with rvec)
   parlist = new_irparam_elemet_ivec(parlist, ascii(str), 12); // id = 12; A string parameter

   p = combine_irparam(parlist); // convert to two vectors of integers and floating point values respectively

// Set-up the block parameters and I/O ports
  Uipar = [ p.ipar ];
  Urpar = [ p.rpar ];
  btype = 1200 + 1; // Reference to the block's type (computational function). Use the same id you are giving via the "libdyn_compfnlist_add" C-function

  insizes=[1,1]; // Input port sizes
  outsizes=[1]; // Output port sizes
  dfeed=[1];  // for each output 0 (no df) or 1 (a direct feedthrough to one of the inputs)
  intypes=[ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_FLOAT]; // datatype for each input port
  outtypes=[ORTD.DATATYPE_FLOAT]; // datatype for each output port

  blocktype = 1; // 1-BLOCKTYPE_DYNAMIC (if block uses states), 2-BLOCKTYPE_STATIC (if there is only a static relationship between in- and output)

  // Create the block
  [sim, blk] = libdyn_CreateBlockAutoConfig(sim, events, btype, blocktype, Uipar, Urpar, insizes, outsizes, intypes, outtypes, dfeed);
  
  // connect the inputs
 [sim,blk] = libdyn_conn_equation(sim, blk, list(in1, in2) ); // connect in1 to port 0 and in2 to port 1

  // connect the ouputs
 [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction




// 
// How to use Shared Objects
// 

function [sim] = RehaMovePro_shObj(sim, events, ObjectIdentifyer, Visibility, par) // PARSEDOCU_BLOCK
// ADD SOME DOCUMENTATION HERE, that will be copied to the scilab help
// 
// This function creates a shared object
// 

  // add a postfix that identifies the type of the shared object
  ObjectIdentifyer = ObjectIdentifyer + ".RehaMovePro_ShObj";


// introduce some parameters that are refered to by id's
//   parameter1 = 12345;
   vec = [par.EMGSampleFrq,2,3];

   // pack all parameters into a structure "parlist"
   parlist = new_irparam_set();

//   parlist = new_irparam_elemet_ivec(parlist, parameter1, 10); // id = 10
   parlist = new_irparam_elemet_ivec(parlist, vec, 11); // vector of integers (double vectors are similar, replace ivec with rvec)
   parlist = new_irparam_elemet_ivec(parlist, ascii(par.DeviceNameStr), 12); // id = 12; A string parameter

   p = combine_irparam(parlist); // convert to two vectors of integers and floating point values respectively

// Set-up the block parameters. There are no I/O ports
  Uipar = [ p.ipar ];
  Urpar = [ p.rpar ];
  btype = 1200 + 10; // Reference to the block's type (computational function). Use the same id you are giving via the "libdyn_compfnlist_add" C-function

  [sim] =  libdyn_CreateSharedObjBlk(sim, btype, ObjectIdentifyer, Visibility, Uipar, Urpar); 
endfunction


function [sim, out] = RehaMovePro_Read(sim, events, ObjectIdentifyer, EMGvecsize, in1, in2) // PARSEDOCU_BLOCK
// ADD SOME DOCUMENTATION HERE, that will be copied to the scilab help
// 
// This function can use a shared object created by ld_Template_shObj
// 
// 
// 

  // add a postfix that identifies the type of the shared object
  ObjectIdentifyer = ObjectIdentifyer + ".RehaMovePro_ShObj";

// introduce some parameters that are refered to by id's
parameter1 = 0;

   // pack all parameters into a structure "parlist"
   parlist = new_irparam_set();

   parlist = new_irparam_elemet_ivec(parlist, parameter1, 10); // id = 10

   p = combine_irparam(parlist); // convert to two vectors of integers and floating point values respectively

// Set-up the block parameters and I/O ports
  Uipar = [ p.ipar ];
  Urpar = [ p.rpar ];
  btype = 1200 + 11; // Reference to the block's type (computational function). Use the same id you are giving via the "libdyn_compfnlist_add" C-function

  insizes=[1,1]; // Input port sizes
  outsizes=[EMGvecsize, EMGvecsize, EMGvecsize, 1, 1]; // Output port sizes
  dfeed=[1];  // for each output 0 (no df) or 1 (a direct feedthrough to one of the inputs)
  intypes=[ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_FLOAT]; // datatype for each input port
  outtypes=[ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_INT32, ORTD.DATATYPE_INT32, ORTD.DATATYPE_INT32]; // datatype for each output port

  blocktype = 1; // 1-BLOCKTYPE_DYNAMIC (if block uses states), 2-BLOCKTYPE_STATIC (if there is only a static relationship between in- and output)

  // Create the block
  [sim, blk] = libdyn_CreateBlockAutoConfig(sim, events, btype, blocktype, Uipar, Urpar, insizes, outsizes, intypes, outtypes, dfeed, ObjectIdentifyer);
  
  // connect the inputs
 [sim,blk] = libdyn_conn_equation(sim, blk, list(in1, in2) ); // connect in1 to port 0 and in2 to port 1

  // connect the ouputs
 [sim,CH1] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
 [sim,CH2] = libdyn_new_oport_hint(sim, blk, 1);   
 [sim,ST] = libdyn_new_oport_hint(sim, blk, 2);   
 [sim,Status] = libdyn_new_oport_hint(sim, blk, 3);   
 [sim,Nsamples] = libdyn_new_oport_hint(sim, blk, 4);  

  out.CH1 = CH1;
  out.CH2 = CH2;
  out.StimulationType = ST;
  out.Status = Status;
  out.Nsamples = Nsamples;
endfunction

function [sim, out] = RehaMovePro_Write(sim, events, ObjectIdentifyer, I, pw, CH) // PARSEDOCU_BLOCK
// ADD SOME DOCUMENTATION HERE, that will be copied to the scilab help
// 
// This function can use a shared object created by ld_Template_shObj
// 
// 
// 

  // add a postfix that identifies the type of the shared object
  ObjectIdentifyer = ObjectIdentifyer + ".RehaMovePro_ShObj";

// introduce some parameters that are refered to by id's
parameter1 = 0;

   // pack all parameters into a structure "parlist"
   parlist = new_irparam_set();

   parlist = new_irparam_elemet_ivec(parlist, parameter1, 10); // id = 10

   p = combine_irparam(parlist); // convert to two vectors of integers and floating point values respectively

// Set-up the block parameters and I/O ports
  Uipar = [ p.ipar ];
  Urpar = [ p.rpar ];
  btype = 1200 + 12; // Reference to the block's type (computational function). Use the same id you are giving via the "libdyn_compfnlist_add" C-function

  insizes=[4,4]; // Input port sizes
  outsizes=[1]; // Output port sizes
  dfeed=[1];  // for each output 0 (no df) or 1 (a direct feedthrough to one of the inputs)
  intypes=[ORTD.DATATYPE_FLOAT, ORTD.DATATYPE_FLOAT]; // datatype for each input port
  outtypes=[ORTD.DATATYPE_INT32]; // datatype for each output port

  blocktype = 1; // 1-BLOCKTYPE_DYNAMIC (if block uses states), 2-BLOCKTYPE_STATIC (if there is only a static relationship between in- and output)

  // Create the block
  [sim, blk] = libdyn_CreateBlockAutoConfig(sim, events, btype, blocktype, Uipar, Urpar, insizes, outsizes, intypes, outtypes, dfeed, ObjectIdentifyer);
  
  // connect the inputs
 [sim,blk] = libdyn_conn_equation(sim, blk, list(I, pw) ); // connect in1 to port 0 and in2 to port 1

  // connect the ouputs
 [sim,Status] = libdyn_new_oport_hint(sim, blk, 0);   

  out.Status = Status;
endfunction

// 
// 
// Interfacing functions are placed in this place
// 
// 
// This is a template from which scilab_loader.sce is automatically produced
// when running the module's makefile.
//
// The placeholder 70101 will be repalced when running the Makefile by the 
// contents of the variable blockid_start in the beginning of the Makefile
// 


// 
// ortd_checkpar types:
// 
//     'Signal' 
//     'SignalList' 
//     'SingleValue' 
//     'Vector'
//     'String'
// 
//  e.g.
// 
//   ortd_checkpar(sim, list('Signal', 'in', in) );
//   ortd_checkpar(sim, list('SingleValue', 'gain', gain) );
// 



function [sim, out] = ld_RPiNeoPixel(sim, events, CH1, CH2, par ) // PARSEDOCU_BLOCK
// 
// RPiNeoPixelBlock - block
//
// CH1 : struct(  *R, *G, *B, *W, Nleds  )
// CH2 : struct(  *R, *G, *B, *W, Nleds  )
//
// par : struct(  )
// 

   // check the input parameters
//   ortd_checkpar(sim, list('String', 'str', str) );
//   ortd_checkpar(sim, list('Signal', 'in1', in1) );
//   ortd_checkpar(sim, list('Signal', 'in2', in2) );

//    ortd_checkpar(sim, list('SingleValue', 'gain', gain) );


// introduce some parameters that are refered to by id's
   parameter1 = 12345;
   vec = [ CH1.Nleds , CH2.Nleds ];

   // pack all parameters into a structure "parlist"
   parlist = new_irparam_set();

   parlist = new_irparam_elemet_ivec(parlist, parameter1, 10); // id = 10
   parlist = new_irparam_elemet_ivec(parlist, vec, 11); // vector of integers (double vectors are similar, replace ivec with rvec)
  // parlist = new_irparam_elemet_ivec(parlist, ascii(str), 12); // id = 12; A string parameter

   p = combine_irparam(parlist); // convert to two vectors of integers and floating point values respectively

// Set-up the block parameters and I/O ports
  Uipar = [ p.ipar ];
  Urpar = [ p.rpar ];
  btype = 70101 + 0; // Reference to the block's type (computational function). Use the same id you are giving via the "libdyn_compfnlist_add" C-function

  insizes=[CH1.Nleds,CH1.Nleds,CH1.Nleds,CH1.Nleds,  CH2.Nleds,CH2.Nleds,CH2.Nleds,CH2.Nleds ]; // Input port sizes
  outsizes=[1]; // Output port sizes
  dfeed=[1];  // for each output 0 (no df) or 1 (a direct feedthrough to one of the inputs)
  intypes=[ORTD.DATATYPE_INT32 * ones(4,1) ;   ORTD.DATATYPE_INT32 * ones(4,1) ]; // datatype for each input port
  outtypes=[ORTD.DATATYPE_FLOAT]; // datatype for each output port

  blocktype = 1; // 1-BLOCKTYPE_DYNAMIC (if block uses states), 2-BLOCKTYPE_STATIC (if there is only a static relationship between in- and output)

  // Create the block
  [sim, blk] = libdyn_CreateBlockAutoConfig(sim, events, btype, blocktype, Uipar, Urpar, insizes, outsizes, intypes, outtypes, dfeed);
  
  // connect the inputs
 [sim,blk] = libdyn_conn_equation(sim, blk, list(  CH1.R, CH1.G, CH1.B, CH1.W,   CH2.R, CH2.G, CH2.B, CH2.W  ) ); // connect in1 to port 0 and in2 to port 1

  // connect the ouputs
 [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction


//
//    Copyright (C) 2010, 2011  Christian Klauer
//
//    This file is part of OpenRTDynamics, the Real Time Dynamic Toolbox
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





// Interfacing functions are placed in this place



function [sim,out] = ld_parameter(sim, events, str, initial_param) // PARSEDOCU_BLOCK
  // Creates a new parameter block that is remotely controlable via TCP
  // It requires the set-up of a libdyn master
  // 
  // str - is a string of the parameter name
  // initial_param - is a vector of the initial parameter set
  // out - is a vectorial signal of size length(initial_param)
  // 
  Nin = 0;
  Nout = 1;

  btype = 14001;
  str = ascii(str);
  nparam = length(initial_param);
  
  ipar = [0, nparam, length(str), str(:)'];
  rpar = [initial_param(:)];
  
  [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
                       insizes=[], outsizes=[nparam], ...
                       intypes=[], outtypes=[ORTD.DATATYPE_FLOAT]  );


//   [sim,blk] = libdyn_conn_equation(sim, blk, inlist);
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction


function [sim] = ld_stream(sim, events, in, str, insize, autoflushInterval, autoflushTimeout, bufferlen) // PARSEDOCU_BLOCK
  // Creates a new stream block that is remotely controlable via TCP
  // It requires the set-up of a libdyn master
  // 
  // str - is a string of the stream name
  // insize is the vector length of the input port
  // [autoflushInterval] how often (in samples) to flush the send buffers
  // [autoflushTimeout] how often to flush the send buffers (time difference) (not implemented)
  // [bufferlen] number of samples within the ringbuffer
  //

  // set some defeult values for optional variables if not present
  if (exists('bufferlen') == 0)
    bufferlen = 1000; // how many vectors should be stored in the ringbuffer
  end

  if (exists('autoflushInterval') == 0)
    autoflushInterval = 1;
  end

  if (exists('autoflushTimeout') == 0)
    autoflushTimeout = 0.1;
  end

  datatype = -1; // FIXME set to FLOAT


  btype = 14001 + 1;
  str = ascii(str);

  
  ipar = [0, insize, datatype, bufferlen, autoflushInterval, length(str), str(:)'];
  rpar = [autoflushTimeout ];
  
  [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
                       insizes=[insize], outsizes=[], ...
                       intypes=[ORTD.DATATYPE_FLOAT], outtypes=[]  );

  // connect input port
  [sim,blk] = libdyn_conn_equation(sim, blk, list(in) );
endfunction

// Interfacing functions are placed in this place





// An example could be the following scilab function. PARSEDOCU_BLOCK is a hint for the documentation generator to generate documentation for this block
function [sim, outlist] = ld_scicosblock(sim, events, inlist, cosblk) // PARSEDOCU_BLOCK
// 
// %PURPOSE: include a scicos block / schematic
//
// inlist  - list() of input ports forwarded to the Scicosblock
// outlist  - list() of output ports forwarded from the Scicosblock
// 
// cosblk - structure as loaded by ortd_getcosblk containg the block parameters 
//
// Hint: There is a  way to compile Xcos-superblocks into C-computational functions.
//       These functions can also be included by ld_scicosblock, but this is experimental
//       and the process for doing so takes some manual steps to perform on the source-code.
// 
// 
// Limitations:
// 
// In the C-structure "scicos_block" there is an entry ozptr that is not initiated 
// correcty by the wrapper in the file "ScicosWrapper.cpp". Only the following steps
// are performed that may be suffiecient for most applications:
// 
//   ozptr[0] = malloc(1000);
//   ozptr[1] = malloc(1000);
//   ozptr[2] = malloc(1000);
//   ozptr[3] = malloc(1000);
//   ozptr[4] = malloc(1000);  
// 
// These fields are used by the generated C-functions for Scicos-Superblocks.
// 

  

  model = cosblk;

//   identstr = model.blockname;
  identstr = model.sim(1); // name of C-comp fn

  printf("Including Scicos Block %s\n", identstr);
  printf("Make sure that the C function %s() is included in the binary used for execution\n", identstr);

  
//    if ( length(cosblk.in) ~= 1 ) then
//      error("Only one input port to the Scicos-Block allowed at the moment");
//    end
//  
//    if ( length(cosblk.out) ~= 1 ) then
//      error("Only one output port to the Scicos-Block allowed at the moment");
//    end
 
 
   Nin = length(cosblk.in);
   Nout = length(cosblk.out);

  
//           if ( irpar_get_ivec(&insizes, ipar, rpar, 10) < 0 ) error = 1 ;
//         if ( irpar_get_ivec(&outsizes, ipar, rpar, 11) < 0 ) error = 1 ;
//         if ( irpar_get_ivec(&intypes, ipar, rpar, 12) < 0 ) error = 1 ;
//         if ( irpar_get_ivec(&outtypes, ipar, rpar, 13) < 0 ) error = 1 ;
//         if ( irpar_get_ivec(&param, ipar, rpar, 18) <  0 ) error = 1 ;
//         
//         int dfeed = param.v[1];
// 
//     
//     if ( irpar_get_ivec(&identstr__, ipar, rpar, 20) < 0 ) error = 1 ;
//     if ( irpar_get_ivec(&block_ipar, ipar, rpar, 21) < 0 ) error = 1 ;
//     if ( irpar_get_rvec(&block_rpar, ipar, rpar, 22) < 0 ) error = 1 ;
//     if ( irpar_get_rvec(&dstate, ipar, rpar, 23) < 0 ) error = 1 ;
//     
  

  dfeed = 1;
  parameters = [dfeed];
  
  parlist = new_irparam_set();

  parlist = new_irparam_elemet_ivec(parlist, cosblk.in, 10);
  parlist = new_irparam_elemet_ivec(parlist, cosblk.out, 11);
  parlist = new_irparam_elemet_ivec(parlist, ORTD.DATATYPE_FLOAT*ones(cosblk.in) , 12); // only float is supported by now
  parlist = new_irparam_elemet_ivec(parlist, ORTD.DATATYPE_FLOAT*ones(cosblk.out) , 13); // only float is supported by now
  parlist = new_irparam_elemet_ivec(parlist, parameters, 18);


  parlist = new_irparam_elemet_ivec(parlist, ascii(identstr), 20);
  parlist = new_irparam_elemet_ivec(parlist, model.ipar, 21);
  parlist = new_irparam_elemet_rvec(parlist, model.rpar, 22);
  parlist = new_irparam_elemet_rvec(parlist, model.dstate, 23);

  blockparam = combine_irparam(parlist);



 btype = 15200 + 0;; // the same id you are giving via the "libdyn_compfnlist_add" C-function
//  [sim,blk] = libdyn_new_block(sim, events, btype, ipar=[ 0, Nin, Nout, length(identstr), ascii(identstr)  ], rpar=[  ], ...
//                   insizes=[Nin], outsizes=[Nout], ...
//                   intypes=[ORTD.DATATYPE_FLOAT], outtypes=[ORTD.DATATYPE_FLOAT]  );



 [sim,blk] = libdyn_new_block(sim, events, btype, ipar=[ blockparam.ipar(:)'  ], rpar=[ blockparam.rpar(:)' ], ...
                  insizes=[  cosblk.in(:)' ], outsizes=[ cosblk.out(:)' ], ...
                  intypes=[ones(1,Nin)*ORTD.DATATYPE_FLOAT], outtypes=[ones(1,Nout)*ORTD.DATATYPE_FLOAT]  );


                  
   [sim,blk] = libdyn_conn_equation(sim, blk, inlist );
 
   outlist = list();
  for i = 0:(Nout-1)
    [sim,out] = libdyn_new_oport_hint(sim, blk, i);   // ith port
    outlist(i+1) = out;
  end

 
 //[sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction




function cosblk=ortd_getcosblk2(blockname, flag, cachefile)  // PARSEDOCU_BLOCK
  //
  // %PURPOSE: Extract information from Scicos block interfacing function macros (*.sci) files
  //
  // pathtoscifile - The interfacing function macro (a *.sci file)
  //   
  // flag - one of 'defaults', 'rundialog' (shows a dialog that asks for the
  //                blocks parameters,
  //               'usecachefile' (prevents to apperance of the dialog)
  //
  // The return value is a structure to be used by ld_scicosblock
  //

  //exec(blockname + '_c.sci');
//   exec(pathtoscifile);

//  Scicos
//
//  Copyright (C) INRIA - METALAU Project <scicos@inria.fr>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
//
// See the file ../license.txt
//

function model=scicos_model(v1,v2,v3,v4,v5,v6,v7,v8,v9,v10,v11,v12,v13,v14,v15,v16,...
                            v17,v18,v19,v20,v21,v22,v23,v24,v25,v26)

  //initialisation de model mlist
  if exists('sim','local')==0 then sim='',end
  if exists('in','local')==0 then in=[],end
  if exists('in2','local')==0 then in2=[],end
  if exists('intyp','local')==0 then intyp=1,end
  if exists('out','local')==0 then out=[],end
  if exists('out2','local')==0 then out2=[],end
  if exists('outtyp','local')==0 then outtyp=1,end
  if exists('evtin','local')==0 then evtin=[],end
  if exists('evtout','local')==0 then evtout=[],end
  if exists('state','local')==0 then state=[],end
  if exists('dstate','local')==0 then dstate=[],end
  if exists('odstate','local')==0 then odstate=list(),end
  if exists('opar','local')==0 then opar=list(),end
  if exists('rpar','local')==0 then rpar=[],end
  if exists('ipar','local')==0 then ipar=[],end
  if exists('blocktype','local')==0 then blocktype='c',end
  if exists('firing','local')==0 then firing=[],end
  if exists('dep_ut','local')==0 then dep_ut=[%f %f],end
  if exists('label','local')==0 then label='',end
  if exists('nzcross','local')==0 then nzcross=0,end
  if exists('nmode','local')==0 then nmode=0,end
  if exists('equations','local')==0 then equations=list(),end

//  model=mlist(['model','sim','in','in2','intyp','out','out2','outtyp',...
//               'evtin','evtout','state','dstate','odstate','rpar','ipar','opar',...
//               'blocktype','firing','dep_ut','label','nzcross',..
//               'nmode','equations'],..
//               sim,in,in2,intyp,out,out2,outtyp,evtin,evtout,state,dstate,odstate,..
//               rpar,ipar,opar,blocktype,firing,dep_ut,label,nzcross,nmode,equations)

			model.sim=sim,..
			model.in=in,..
			model.in2=in2,..
			model.intyp=intyp,..
			model.out=out,..
			model.out2=out2,..
			model.outtyp=outtyp,..
//			model.evtin=clkinput,..
//			model.evtout=clkoutput,..
			model.firing=firing,..
			model.state=state,..
			model.dstate=dstate,..
			model.odstate=odstate,..
			model.rpar=rpar,..
			model.ipar=ipar,..
			model.opar=opar,..
			model.blocktype='c',..
			model.dep_ut=dep_ut,..
			model.nzcross=nzcross,..
			model.nmode=nmode  
endfunction


  function [model,graphics,ok]=check_io(model,graphics,in,out,clkin,clkout,in_implicit,out_implicit)
    // no actual check, just a copy into the model structure
    ok=%t


    model.evtin=clkin
    model.evtout=clkout
    model.in = in;
    model.out = out;
    model.evkin = clkin;
    model.evout = clkout;
  endfunction


  function x=standard_define( a, b, c, d )
    x.a =  a;
    x.b = b; // model
    x.c = c(:); // exprs
    x.d = d;

    x.a =  a;
    x.model = b; // model
    x.graphics.exprs = c(:); // exprs
    x.d = d;
  endfunction

//   printf("Parameters: blockname = %s, flag = %s, cachefile %s, \n", blockname, flag, cachefile);  

  if (flag == 'usecachefile') then
//       [lhs,rhs]=argn(0);
      rhs=3;
      
      if rhs >= 3 then
        try
          load(cachefile);  // ideally results in a new variable X, which is a structure
          printf("Using cachefile %s\n", cachefile);
          
          cosblk = X.model;
          cosblk.timestamp = getdate();
          cosblk.blockname = blockname;

        catch
          1;
        end
        1;
      else
        error("No cachefile was specified");
      end      
  end
 
  if (flag == 'rundialog') then
  
    // overwrite a potenially already available variable X, which does not belong this code.
    X=0; // make typeof X to be constant
  
    // check for cached cosblk
    [lhs,rhs]=argn(0);
      
    rhs = 3; // always assume that the thirtd parameter is there

      if rhs >= 3 then
        try
          load(cachefile);  // ideally results in a new variable X, which is a structure
          printf("Using cachefile %s\n", cachefile);
        catch
          printf("No cachefile is loaded\n");
        end
        1;
      end
      
    // if not available, call standard_define
    if (typeof(X) == 'constant') then // no new X was loaded, call macro for default values
      printf("loading block''s standard parameters\n");
    
      definecommand = "" + blockname + "(job=''define'',arg1=0,arg2=0);";
      X = eval(definecommand);
//       cosblk = X.b;
    end

    

//     arg1.graphics.exprs = X.graphics.exprs;
    arg1.graphics = X.graphics;
    arg1.model = X.model;
    definecommand = "" + blockname + "(job=''set'',arg1,arg2=0);";
    X = eval(definecommand);

    
    cosblk = X.model;
    cosblk.timestamp = getdate();
    cosblk.blockname = blockname;
    
    // print info
    printf("I/O of Scicosblock:\n  insizes=%s,\n  outsizes=%s\n", sci2exp( cosblk.in(:)' ), sci2exp( cosblk.out(:)' ) );

    // check for cached cosblk
//     [lhs,rhs]=argn(0) 
    rhs = 3; // always assume that the thirtd parameter is there

//     printf("rhs... = %d\n", rhs);
//     if rhs >= 3 then
      printf("Saving to cachefile %s\n", cachefile);

      if getversion() == "scilab-5.3.3" // Grrrr!
        save(cachefile, X);
      else
        save(cachefile, 'X'); // newer scilab versions
      end
//     end
      

  end
//  clear X;
endfunction




function cosblk=ortd_getcosblk(blockname, pathtoscifile)  // PARSEDOCU_BLOCK
  //
  // %PURPOSE: Extract information from Scicos block interfacing function macros (*.sci) files
  //
  // pathtoscifile - The interfacing function macro (a *.sci file)
  //

  
  // check if pathtoscifile is a function

  //exec(blockname + '_c.sci');
//   if (typeof(pathtoscifile) == 'string') then
   if pathtoscifile ~= "" then
     exec(pathtoscifile);
   end
//   end

  function model = scicos_model(sim,..
			in,..
			in2,..
			intyp,..
			out,..
			out2,..
			outtyp,..
			evtin,..
			evtout,..
			firing,..
			state,..
			dstate,..
			odstate,..
			rpar,..
			ipar,..
			opar,..
			blocktype,..
			dep_ut,..
			nzcross,..
			nmode )
			
			model.sim=sim,..
			model.in=in,..
			model.in2=in2,..
			model.intyp=intyp,..
			model.out=out,..
			model.out2=out2,..
			model.outtyp=outtyp,..
			model.evtin=clkinput,..
			model.evtout=clkoutput,..
			model.firing=firing,..
			model.state=x,..
			model.dstate=Z,..
			model.odstate=odstate,..
			model.rpar=rpar,..
			model.ipar=ipar,..
			model.opar=opar,..
			model.blocktype='c',..
			model.dep_ut=dep_ut,..
			model.nzcross=nzcross,..
			model.nmode=nmode           
			
  endfunction

  function x=standard_define( a, b, c, d )
    x.a =  a;
    x.b = b;
    x.c = c;
    x.d = d;
  endfunction

  //definecommand = "[x,y,typ]=" + blockname + "_c(job=''define'',arg1=0,arg2=0);";
  definecommand = "" + blockname + "_c(job=''define'',arg1=0,arg2=0);";
  X = eval(definecommand);
  cosblk = X.b;
  cosblk.timestamp = getdate();
  cosblk.blockname = blockname;
  clear X;

endfunction

// function cosblk=ortd_getcosblk2(blockname, pathtoscifile)  // PARSEDOCU_BLOCK
//   //
//   // %PURPOSE: Extract information from Scicos block interfacing function macros (*.sci) files
//   //
//   // pathtoscifile - The interfacing function macro (a *.sci file)
//   //
// 
// 
//   //exec(blockname + '_c.sci');
//   exec(pathtoscifile);
// 
//   function model = scicos_model(sim,..
// 			in,..
// 			in2,..
// 			intyp,..
// 			out,..
// 			out2,..
// 			outtyp,..
// 			evtin,..
// 			evtout,..
// 			firing,..
// 			state,..
// 			dstate,..
// 			odstate,..
// 			rpar,..
// 			ipar,..
// 			opar,..
// 			blocktype,..
// 			dep_ut,..
// 			nzcross,..
// 			nmode )
// 			
// 			model.sim=sim,..
// 			model.in=in,..
// 			model.in2=in2,..
// 			model.intyp=intyp,..
// 			model.out=out,..
// 			model.out2=out2,..
// 			model.outtyp=outtyp,..
// 			model.evtin=clkinput,..
// 			model.evtout=clkoutput,..
// 			model.firing=firing,..
// 			model.state=x,..
// 			model.dstate=Z,..
// 			model.odstate=odstate,..
// 			model.rpar=rpar,..
// 			model.ipar=ipar,..
// 			model.opar=opar,..
// 			model.blocktype='c',..
// 			model.dep_ut=dep_ut,..
// 			model.nzcross=nzcross,..
// 			model.nmode=nmode           
// 			
//   endfunction
// 
//   function x=standard_define( a, b, c, d )
//     x.a =  a;
//     x.b = b;
//     x.c = c;
//     x.d = d;
//   endfunction
// 
//   definecommand = "" + blockname + "(job=''define'',arg1=0,arg2=0);";
//   X = eval(definecommand);
//   cosblk = X.b;
//   cosblk.timestamp = getdate();
//   cosblk.blockname = blockname;
//   clear X;
// 
// endfunction
// 
// model=ortd_getcosblk(blockname="SuperBlock");
// save(blockname+"_scicosblockcfg.dat", model);




// Interfacing functions are placed into this place

// printf("Hey, this is the ORTD-Toolbox\n");

function [sim, out] = ld_scilab(sim, events, in, invecsize, outvecsize, init_cmd, calc_cmd, destruct_cmd, scilab_path) // PARSEDOCU_BLOCK
// %PURPOSE: Block for interfacing scilab
//
// in *+(invecsize) - input        scilab_interf.invec%d = [ .... ];
// out *+(outvecsize) - output        scilab_interf.outvec%d = [ .... ];
// scilab_path - Path to scilab5 executable
// 
// out = calc_cmd        scilab_interf.outvec%d = calc_cmd(scilab_interf.invec%d);
//
// 


  invecno=1; outvecno=1;

  init_cmd = ascii(init_cmd);
  calc_cmd = ascii(calc_cmd);
  destruct_cmd = ascii(destruct_cmd);
  scilab_path = ascii(scilab_path);
  
  btype = 22000;
  ipar = [invecsize; outvecsize; invecno; outvecno; 0; 0; 0; 0; 0; 0; 0; length(init_cmd); length(calc_cmd); ...
                   length(destruct_cmd); length(scilab_path); (init_cmd)';...
                   (calc_cmd)'; (destruct_cmd)'; (scilab_path)'; 0; ]; rpar = [];
  
//   pause;
  
  [sim,blk] = libdyn_new_block(sim, events, btype, ipar, rpar, ...
                   insizes=[invecsize], outsizes=[outvecsize], ...
                   intypes=[ORTD.DATATYPE_FLOAT], outtypes=[ORTD.DATATYPE_FLOAT]  );

  // libdyn_conn_equation connects multiple input signals to blocks
  [sim,blk] = libdyn_conn_equation(sim, blk, list(in) );
  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction


// Computaional function example:
// 
//function [block]=sample_comp_fn( block, flag )
//  select flag
//    case 0 // update
//      printf("update states\n");
//
//    case 1 // output
//      printf("update outputs\n");
//      outvec = [1:6]';
//
//      block.outptr(1) = outvec;
//
//    case 4 // init
//      printf("init\n");
//
//    case 5 // terminate
//      printf("terminate\n");
//
//    case 10 // configure
//      printf("configure\n");
//      block.invecsize = 5;
//      block.outvecsize = 6;
//
//  end
//endfunction



function [sim, out] = ld_scilab2(sim, events, in, comp_fn, include_scilab_fns, scilab_path) // PARSEDOCU_BLOCK
// %PURPOSE: Block for calling a computational function written in scilab
//
// A nicer interface to scilab. (See modules/scilab/demo/scilab_demo.sce for an example)
//
// in *+(invecsize) - input
// out *+(outvecsize) - output
// 
// out = calc_cmd
//
//
// comp_fn - scilab function that is executed online. Prototype:
//           function [block]=sample_comp_fn( block, flag )
//
//           flags are: 1 calc outputs, 4 initialise, 5 terminate, 10 configure I/O
//
//           For further details have a look at the example.
//
// include_scilab_fns - unused for now. Set to list()
// scilab_path - Path to scilab5 executable. Use "BUILDIN_PATH" if you do not have special needs.
//
// NOTE: For now the flag "update states" is not called; Also a "reset states" flag is required.
//
//

  function [result] = check_structure( str )
    // give something like str="a.b" to see wheter this would lead to an error

    result = %T;
    err = execstr(str, 'errcatch');
    if (err ~= 0) then
      result = %F;
    end
  endfunction

  block.dummy = 0;

  // Get the number of I/O
  block = comp_fn(block, 10);

  if check_structure("block.invecsize") == %F then
    error("comp_fn did not return block.invecsize\n");
  end

  if check_structure("block.outvecsize") == %F then
    error("comp_fn did not return block.outvecsize\n");
  end

  invecsize = block.invecsize;
  outvecsize = block.outvecsize;


  init_cmd="";
  LF = char(10);

  // look for any global variables; Could be used to build a set of variables that are then transfered
  // to the on-line scilab instance.
//   vars = macrovar(comp_fn); vars(3);  // however it also shows the nested functions

  // make source code from the computational function
  function_string = fun2string(comp_fn);
  function_string(1) = strsubst(function_string(1), "ans", "comp_fn"); // rename the function

  for i=1:length(length(function_string))
    init_cmd = init_cmd + function_string(i) + LF; // concate lines and add a line feed character
  end

// pause;

  // add all functions from include_scilab_fns
  for j=1:(length(include_scilab_fns)/2)


    fn = include_scilab_fns(  (j-1)*2 + 1  );
    tmp = fun2string(fn);

    fnname = include_scilab_fns( (j-1)*2 + 2 );
    tmp(1) = strsubst(tmp(1), "ans", fnname); // rename the function 

    for i=1:length(length(tmp))
      init_cmd = init_cmd + tmp(i) + LF; // concate lines and add a line feed character
    end
  end

// disp(init_cmd);

//   pause;


  // Prepare initialisation string, which contains the comp_fn in form of its source code
  init_str = "" + LF + ...
             "block.invecsize=" + string(invecsize) + ";" + LF + ...
             "block.outvecsize=" + string(outvecsize) + ";" + LF + ...
             "block=comp_fn(block, 4);" + LF + ...
             "printf(''scilab computational function initialised\n''); " + LF + ...
             "block.inptr = list(1);" + LF + ...
             "block.outptr = list(1);" + LF;

  init_cmd = init_cmd + init_str;

  // Calc command
  calc_cmd = "block.inptr(1) = scilab_interf.invec1;" + ...
             "block=comp_fn(block, 1); " + ...
             "scilab_interf.outvec1 = block.outptr(1); " + ... // all in one line
             "" + LF;

  // command for destruction
  destruct_cmd = "" + LF + ...
             "block=comp_fn(block, 5);" + LF + ...
             "printf(''scilab computational function destructed\n''); " + LF + ...
             "clear block;" + LF;

//  fd = mopen('init_cmd.txt','wt');  mputl(init_cmd,fd);  mclose(fd);
//  fd = mopen('calc_cmd.txt','wt');  mputl(calc_cmd,fd);  mclose(fd);
//  fd = mopen('destruct_cmd.txt','wt');  mputl(destruct_cmd,fd);  mclose(fd);

//pause;

  // auf bereits bekannte funktion zurückführen
  [sim, out] = ld_scilab(sim, events, in, invecsize, outvecsize,  ...
    init_cmd, calc_cmd, destruct_cmd, scilab_path);

endfunction

function [sim, out] = ld_scilab3(sim, events, in, comp_fn, include_scilab_fns, InitStr, scilab_path) // PARSEDOCU_BLOCK
// %PURPOSE: Block for calling a computational function written in scilab
//
// A nicer interface to scilab. (See modules/scilab/demo/scilab_demo.sce for an example)
//
// in *+(invecsize) - input
// out *+(outvecsize) - output
// InitStr - execute str in the begining. e.g. to define global variables
// 
// out = calc_cmd
//
//
// comp_fn - scilab function that is executed online. Prototype:
//           function [block]=sample_comp_fn( block, flag )
//
//           flags are: 1 calc outputs, 4 initialise, 5 terminate, 10 configure I/O
//
//           For further details have a look at the example.
//
// include_scilab_fns - unused for now. Set to list()
// scilab_path - Path to scilab5 executable. Use "BUILDIN_PATH" if you do not have special needs.
// 
// Nice feature: ld_scilab3 will look for variables that are not defined within comp_fn
//               and will try to transfer these variables from the Scilab instance running at
//               the development host to the on-line running Scilab instance.
//               This works only for single value variables only, though.
//
// NOTE: For now the flag "update states" is not called; Also a "reset states" flag is required.
//
//


   // check the input parameters
   ortd_checkpar(sim, list('String', 'InitStr', InitStr) );
   ortd_checkpar(sim, list('String', 'scilab_path', scilab_path) );
   // FIXME: add missing checks
   ortd_checkpar(sim, list('Signal', 'in', in) );

  function [result] = check_structure( str )
    // give something like str="a.b" to see wheter this would lead to an error

    result = %T;
    err = execstr(str, 'errcatch');
    if (err ~= 0) then
      result = %F;
    end
  endfunction

  block.dummy = 0;

  // Get the number of I/O
  block = comp_fn(block, 10);

  if check_structure("block.invecsize") == %F then
    error("comp_fn did not return block.invecsize\n");
  end

  if check_structure("block.outvecsize") == %F then
    error("comp_fn did not return block.outvecsize\n");
  end

  invecsize = block.invecsize;
  outvecsize = block.outvecsize;

  // look for any global variables; Could be used to build a set of variables that are then transfered
  // to the on-line scilab instance.
  vars = macrovar(comp_fn);   V=vars(3); // however it also shows the nested functions
  printf("ld_scilab3: The following variables will be forwarded to the on-line Scilab instance:\n");

  ParamDefineStr = ""; // Code for the on-line Scilab to define the variables

  // go through all variables and build a list of all single value variables
  for i=1:size(V,1)
    
    tmp=0; 
     try
       estr = "ortd_checkpar(sim, list(''SingleValue'', '' '', " + V(i)  + " ) ); "
// printf("exec : %s\n", estr);
       execstr(estr); // the aborts the execution between try and catch, if V(i) is not a single value

       // get the value of the variable named V(i)
       estr = "tmp=" + V(i) + ";";
       // printf("exec : %s\n", estr);
       execstr(estr); // tmp=<V(i)>

       // printf("exec finished\n");
       printf(string(i) + ") " + V(i) + " = " + sprintf("%20.30f",tmp) + "\n");  // print an element of the list
       ParamDefineStr = ParamDefineStr + sprintf( V(i) + "=" + sprintf("%20.30f",tmp) + ";\n"); // build a list of assignment commands
     catch
       null;
       printf("NOTE: The variable named %s cannot be forwarded by now!\n", V(i));
     end
  end

//    printf("The command that will be executed on-line is:\n"); disp(ParamDefineStr);
  InitStr = ParamDefineStr + ";\n" + InitStr;

  init_cmd="";
  LF = char(10);

  // make source code from the computational function
  function_string = fun2string(comp_fn);
  function_string(1) = strsubst(function_string(1), "ans", "comp_fn"); // rename the function

  for i=1:length(length(function_string))
    init_cmd = init_cmd + function_string(i) + LF; // concate lines and add a line feed character
  end

  // add all functions from include_scilab_fns
  for j=1:(length(include_scilab_fns)/2)
    fn = include_scilab_fns(  (j-1)*2 + 1  );
    tmp = fun2string(fn);

    fnname = include_scilab_fns( (j-1)*2 + 2 );
    tmp(1) = strsubst(tmp(1), "ans", fnname); // rename the function 

    for i=1:length(length(tmp))
      init_cmd = init_cmd + tmp(i) + LF; // concate lines and add a line feed character
    end
  end



  // Prepare initialisation string, which contains the comp_fn in form of its source code
  init_str = "" + LF + ...
             "block.invecsize=" + string(invecsize) + ";" + LF + ...
             "block.outvecsize=" + string(outvecsize) + ";" + LF + ...
	      InitStr + ";" + LF + ...
             "block=comp_fn(block, 4);" + LF + ...
             "printf(''scilab computational function initialised\n''); " + LF + ...
             "block.inptr = list(1);" + LF + ...
             "block.outptr = list(1);" + LF;

  init_cmd = init_cmd + init_str;

  // Calc command
  calc_cmd = "block.inptr(1) = scilab_interf.invec1;" + ...
             "block=comp_fn(block, 1); " + ...
             "scilab_interf.outvec1 = block.outptr(1); " + ... // all in one line
             "" + LF;

  // command for destruction
  destruct_cmd = "" + LF + ...
             "block=comp_fn(block, 5);" + LF + ...
             "printf(''scilab computational function destructed\n''); " + LF + ...
             "clear block;" + LF;

//  fd = mopen('init_cmd.txt','wt');  mputl(init_cmd,fd);  mclose(fd);
//  fd = mopen('calc_cmd.txt','wt');  mputl(calc_cmd,fd);  mclose(fd);
//  fd = mopen('destruct_cmd.txt','wt');  mputl(destruct_cmd,fd);  mclose(fd);

//pause;

  // auf bereits bekannte funktion zurückführen
  [sim, out] = ld_scilab(sim, events, in, invecsize, outvecsize,  ...
    init_cmd, calc_cmd, destruct_cmd, scilab_path);

endfunction



function [sim, out] = ld_scilab4(sim, events, in, invecsize, outvecsize, comp_fn, ForwardVars, par) // PARSEDOCU_BLOCK
// %PURPOSE: Block for calling a computational function written in scilab
//
// A nicer interface to scilab. (See modules/scilab/demo/scilab_demo.sce for an example)
//
// in *+(invecsize) - input
// out *+(outvecsize) - output
// 
// out = calc_cmd
//
//
// comp_fn - scilab function that is executed online. Prototype:
//           function [block]=sample_comp_fn( block, flag )
//
//           flags are: 1 calc outputs, 4 initialise, 5 terminate
//
//           For further details have a look at the example.
//
// ForwardVars - %t or %f Forward variables to the online Scilab instance
// 
// The structure par must contain at least the following elements:
// 
// include_scilab_fns - list( fn1, "fn1", fn2, "fn2" ) , whereby fn1 and fn2 stand for Scilab functions
// scilab_path - Path to scilab5 executable. Use "BUILDIN_PATH" if you do not have special needs.
// InitStr - execute str in the begining. e.g. to define global variables
// 
// Nice feature: ld_scilab4 will look for variables that are not defined within comp_fn, if ForwardVars = %t
//               and will try to transfer these variables from the Scilab instance running at
//               the development host to the on-line running Scilab instance.
//               This works only for single value variables only, though.
//
// NOTE: For now the flag "update states" is not called; Also a "reset states" flag is required.
//
//

  // additional parameters
  include_scilab_fns = par.include_scilab_fns;
  InitStr = par.InitStr;
  scilab_path = par.scilab_path;


   // check the input parameters
   ortd_checkpar(sim, list('String', 'InitStr', InitStr) );
   ortd_checkpar(sim, list('String', 'scilab_path', scilab_path) );
   // FIXME: add missing checks
   ortd_checkpar(sim, list('Signal', 'in', in) );

  function [result] = check_structure( str )
    // give something like str="a.b" to see wheter this would lead to an error

    result = %T;
    err = execstr(str, 'errcatch');
    if (err ~= 0) then
      result = %F;
    end
  endfunction


// 
// Rev ld_scilab4: removed I/O configuration by the scilab comp_fn
// 

//   block.dummy = 0;
// 
//   // Get the number of I/O
//   block = comp_fn(block, 10);
// 
//   if check_structure("block.invecsize") == %F then
//     error("comp_fn did not return block.invecsize\n");
//   end
// 
//   if check_structure("block.outvecsize") == %F then
//     error("comp_fn did not return block.outvecsize\n");
//   end
// 
//   invecsize = block.invecsize;
//   outvecsize = block.outvecsize;


  if ForwardVars == %t then
    // look for any global variables; Could be used to build a set of variables that are then transfered
    // to the on-line scilab instance.
    vars = macrovar(comp_fn);   V=vars(3); // however it also shows the nested functions
    printf("ld_scilab3: The following variables will be forwarded to the on-line Scilab instance:\n");

    ParamDefineStr = ""; // Code for the on-line Scilab to define the variables

    // go through all variables and build a list of all single value variables
    for i=1:size(V,1)
      
      tmp=0; 
      try
	estr = "ortd_checkpar(sim, list(''SingleValue'', '' '', " + V(i)  + " ) ); "
  // printf("exec : %s\n", estr);
	execstr(estr); // the aborts the execution between try and catch, if V(i) is not a single value

	// get the value of the variable named V(i)
	estr = "tmp=" + V(i) + ";";
	// printf("exec : %s\n", estr);
	execstr(estr); // tmp=<V(i)>

	// printf("exec finished\n");
	printf(string(i) + ") " + V(i) + " = " + sprintf("%20.30f",tmp) + "\n");  // print an element of the list
	ParamDefineStr = ParamDefineStr + sprintf( V(i) + "=" + sprintf("%20.30f",tmp) + ";\n"); // build a list of assignment commands
      catch
	null;
	printf("NOTE: The variable named %s cannot be forwarded by now!\n", V(i));
      end
    end

  else
    ParamDefineStr = "";
  end


    LF = char(10);
  //    printf("The command that will be executed on-line is:\n"); disp(ParamDefineStr);
    InitStr = InitStr + ParamDefineStr + ";" + LF;

    CompFnStr="";

    // make source code from the computational function
    function_string = fun2string(comp_fn);
    function_string(1) = strsubst(function_string(1), "ans", "comp_fn"); // rename the function

    for i=1:length(length(function_string))
      CompFnStr = CompFnStr + function_string(i) + LF; // concate lines and add a line feed character
    end

    // add all functions from include_scilab_fns
    for j=1:(length(include_scilab_fns)/2)
      fn = include_scilab_fns(  (j-1)*2 + 1  );
      tmp = fun2string(fn);

      fnname = include_scilab_fns( (j-1)*2 + 2 );
      tmp(1) = strsubst(tmp(1), "ans", fnname); // rename the function 

      for i=1:length(length(tmp))
	CompFnStr = CompFnStr + tmp(i) + LF; // concate lines and add a line feed character
      end
    end



  // Prepare initialisation string, which contains the comp_fn in form of its source code
  init_str = "" + LF + ...
             "block.invecsize=" + string(invecsize) + ";" + LF + ...
             "block.outvecsize=" + string(outvecsize) + ";" + LF + ...
	      InitStr + ";" + LF + ...
             "block=comp_fn(block, 4);" + LF + ...
             "printf(''scilab computational function initialised\n''); " + LF + ...
             "block.inptr = list(1);" + LF + ...
             "block.outptr = list(1);" + LF;

  init_cmd = CompFnStr + init_str;



//   disp(init_cmd);

  // Calc command
  calc_cmd = "block.inptr(1) = scilab_interf.invec1;" + ...
             "block=comp_fn(block, 1); " + ...
             "scilab_interf.outvec1 = block.outptr(1); " + ... // all in one line
             "" + LF;

  // command for destruction
  destruct_cmd = "" + LF + ...
             "block=comp_fn(block, 5);" + LF + ...
             "printf(''scilab computational function destructed\n''); " + LF + ...
             "clear block;" + LF;

//  fd = mopen('init_cmd.txt','wt');  mputl(init_cmd,fd);  mclose(fd);
//  fd = mopen('calc_cmd.txt','wt');  mputl(calc_cmd,fd);  mclose(fd);
//  fd = mopen('destruct_cmd.txt','wt');  mputl(destruct_cmd,fd);  mclose(fd);


  // auf bereits bekannte funktion zurückführen
  [sim, out] = ld_scilab(sim, events, in, invecsize, outvecsize,  ...
    init_cmd, calc_cmd, destruct_cmd, scilab_path);

endfunction


// Interfacing functions are placed in this place


// ADJUST HERE: Add your interfacing functions here

// Constants for describing thread types
//   MOVED TO libdyn.sci as a workaround
// ORTD.ORTD_RT_REALTIMETASK = 1;
// ORTD.ORTD_RT_NORMALTASK = 2;



function [sim, out] = ld_synctimer(sim, events, in) // PARSEDOCU_BLOCK
// %PURPOSE: Timer for synchronisation of a async simulation 
//
// To beused within an async nested schematic for introducing variable sample times (EXPERIMENTAL)
// 
// For an example see modules/synchronisation/demo/timed_thread.sce
// 
// This is obsolete and will be removed. Use ld_ClockSync instead.
// 


 btype = 15100 + 0; //
 [sim,blk] = libdyn_new_block(sim, events, btype, ipar=[ 0  ], rpar=[ ], ...
                  insizes=[1], outsizes=[1], ...
                  intypes=[ORTD.DATATYPE_FLOAT], outtypes=[ORTD.DATATYPE_FLOAT]  );

 [sim,blk] = libdyn_conn_equation(sim, blk, list(in) );
 [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction


function [sim, out] = ld_ClockSync(sim, events, in) // PARSEDOCU_BLOCK
// %PURPOSE: Timer for synchronisation of a async simulation 
//
// To beused within an async nested schematic for introducing variable sample times (EXPERIMENTAL)
// 
// For an example see modules/synchronisation/demo/timed_thread.sce
// 
// New version of ld_synctimer (was experimental), which will be removed.
// 

// introduce some parameters that are refered to by id's

   // pack all parameters into a structure "parlist"
   parlist = new_irparam_set();

//    parlist = new_irparam_elemet_ivec(parlist, parameter1, 10); // id = 10
//    parlist = new_irparam_elemet_ivec(parlist, vec, 11); // vector of integers (double vectors are similar, replace ivec with rvec)
//    parlist = new_irparam_elemet_ivec(parlist, ascii(str), 12); // id = 12; A string parameter

   p = combine_irparam(parlist); // convert to two vectors of integers and floating point values respectively

// Set-up the block parameters and I/O ports
  Uipar = [ p.ipar ];  Urpar = [ p.rpar ];

  // Reference to the block's type (computational function). Use the same id you are giving via the "libdyn_compfnlist_add" C-function
  btype = 15100 + 2; //

  insizes=[1]; // Input port sizes
  outsizes=[1]; // Output port sizes
  dfeed=[1];  // for each output 0 (no df) or 1 (a direct feedthrough to one of the inputs)
  intypes=[ORTD.DATATYPE_FLOAT]; // datatype for each input port
  outtypes=[ORTD.DATATYPE_FLOAT]; // datatype for each output port

  blocktype = 1; // 1-BLOCKTYPE_DYNAMIC (if block uses states), 2-BLOCKTYPE_STATIC (if there is only a static relationship between in- and output)

  // Create the block
  [sim, blk] = libdyn_CreateBlockAutoConfig(sim, events, btype, blocktype, Uipar, Urpar, insizes, outsizes, intypes, outtypes, dfeed);
  
  // connect the inputs
 [sim,blk] = libdyn_conn_equation(sim, blk, list(in) ); // connect in1 to port 0 and in2 to port 1

  // connect the ouputs
 [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction




function [sim, out] = ld_clock(sim, events) // PARSEDOCU_BLOCK
//
// %PURPOSE: get current system time in [s]
//
// out * - time
// 
// 


  btype = 15100 + 1;
  [sim,blk] = libdyn_new_block(sim, events, btype, ipar=[ 0 ], rpar=[  ], ...
                   insizes=[], outsizes=[1], ...
                   intypes=[], outtypes=[ORTD.DATATYPE_FLOAT]  );


  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction









function [sim] = ld_notification_shObj(sim, events, ObjectIdentifyer, Visibility) // PARSEDOCU_BLOCK
// 
// Thread notification slot
//
// 
// 
// 
// EXPERIMENTAL
// 


  // add a postfix that identifies the type of the shared object
  ObjectIdentifyer = ObjectIdentifyer + ".ThreadNotifications_shObj";



   // pack all parameters into a structure "parlist"
   parlist = new_irparam_set();

//    parlist = new_irparam_elemet_ivec(parlist, UDPPort, 10); // id = 10
//    parlist = new_irparam_elemet_ivec(parlist, ascii(hostname), 11); // id = 11; A string parameter

   p = combine_irparam(parlist); // convert to two vectors of integers and floating point values respectively

// Set-up the block parameters. There are no I/O ports
  Uipar = [ p.ipar ];
  Urpar = [ p.rpar ];
  btype = 15100 + 110; // Reference to the block's type (computational function). Use the same id you are giving via the "libdyn_compfnlist_add" C-function

  [sim] =  libdyn_CreateSharedObjBlk(sim, btype, ObjectIdentifyer, Visibility, Uipar, Urpar); 
endfunction






function [sim, signal ] = ld_RecvNotifications(sim, events, ObjectIdentifyer) // PARSEDOCU_BLOCK
// 
// Receiver for Thread notifications
//
// signal *, ORTD.DATATYPE_INT32 - The signal that was received
// 
// This is a simulation-synchronising Block. Everytime a notification is received,
// the simulation that contains this blocks goes on for one step.
// 
// Notifications can be send from other threads by using ld_ThreadNotify
// 
// EXPERIMENTAL
// 

  printf("Synchronising simulation to thread-notifications\n");

  // add a postfix that identifies the type of the shared object
  ObjectIdentifyer = ObjectIdentifyer + ".ThreadNotifications_shObj";


   // pack all parameters into a structure "parlist"
   parlist = new_irparam_set();

//    parlist = new_irparam_elemet_ivec(parlist, outsize, 10); // id = 10
//    parlist = new_irparam_elemet_ivec(parlist, outtype, 11); // id = 11

   p = combine_irparam(parlist); // convert to two vectors of integers and floating point values respectively

// Set-up the block parameters and I/O ports
  Uipar = [ p.ipar ];
  Urpar = [ p.rpar ];
  btype = 15100 + 111; // Reference to the block's type (computational function). Use the same id you are giving via the "libdyn_compfnlist_add" C-function

  insizes=[]; // Input port sizes
  outsizes=[1]; // Output port sizes
  dfeed=[1];  // for each output 0 (no df) or 1 (a direct feedthrough to one of the inputs)
  intypes=[]; // datatype for each input port
  outtypes=[ORTD.DATATYPE_INT32]; // datatype for each output port

  blocktype = 1; // 1-BLOCKTYPE_DYNAMIC (if block uses states), 2-BLOCKTYPE_STATIC (if there is only a static relationship between in- and output)

  // Create the block
  [sim, blk] = libdyn_CreateBlockAutoConfig(sim, events, btype, blocktype, Uipar, Urpar, insizes, outsizes, intypes, outtypes, dfeed, ObjectIdentifyer);
  
//   // connect the inputs
//  [sim,blk] = libdyn_conn_equation(sim, blk, list(in) ); // connect in1 to port 0 and in2 to port 1

   // connect the ouputs
  [sim,signal] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction


function [sim] = ld_ThreadNotify(sim, events, ObjectIdentifyer, signal) // PARSEDOCU_BLOCK
// 
// Thread Notify
//
// signal *, ORTD.DATATYPE_INT32 - the signal to send. If zero no signal is send
// 
// EXPERIMENTAL
// 


  // add a postfix that identifies the type of the shared object
  ObjectIdentifyer = ObjectIdentifyer + ".ThreadNotifications_shObj";


   // pack all parameters into a structure "parlist"
   parlist = new_irparam_set();

//    parlist = new_irparam_elemet_ivec(parlist, insize, 10); // id = 10
//    parlist = new_irparam_elemet_ivec(parlist, intype, 11); // id = 11  
//    parlist = new_irparam_elemet_ivec(parlist, ascii(hostname), 13); // id = 11; A string parameter
   

   p = combine_irparam(parlist); // convert to two vectors of integers and floating point values respectively

// Set-up the block parameters and I/O ports
  Uipar = [ p.ipar ];
  Urpar = [ p.rpar ];
  btype = 15100 + 112; // Reference to the block's type (computational function). Use the same id you are giving via the "libdyn_compfnlist_add" C-function

  insizes=[1]; // Input port sizes
  outsizes=[]; // Output port sizes
  dfeed=[1];  // for each output 0 (no df) or 1 (a direct feedthrough to one of the inputs)
  intypes=[ ORTD.DATATYPE_INT32  ]; // datatype for each input port
  outtypes=[]; // datatype for each output port

  blocktype = 1; // 1-BLOCKTYPE_DYNAMIC (if block uses states), 2-BLOCKTYPE_STATIC (if there is only a static relationship between in- and output)

  // Create the block
  [sim, blk] = libdyn_CreateBlockAutoConfig(sim, events, btype, blocktype, Uipar, Urpar, insizes, outsizes, intypes, outtypes, dfeed, ObjectIdentifyer);
  
  // connect the inputs
 [sim,blk] = libdyn_conn_equation(sim, blk, list(signal) ); // connect in1 to port 0 and in2 to port 1

//   // connect the ouputs
//  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction





// Interfacing functions are placed in this place


// ADJUST HERE: Add your interfacing functions here

// An example could be the following scilab function. PARSEDOCU_BLOCK is a hint for the documentation generator to generate documentation for this block
//function [sim, out] = ld_abs(sim, events, in) // PARSEDOCU_BLOCK
//// ADD SOME DOCUMENTATION HERE
//// abs - block
////
//// in * - input
//// out * - output
//// 
//// out = abs(in)
//// 
//
//  btype = 60001 + 7; // the same id you are giving via the "libdyn_compfnlist_add" C-function
//  [sim,blk] = libdyn_new_block(sim, events, btype, ipar=[ <ADD INTEGERS HERE>  ], rpar=[ <ADD FLOATING POITN VALUES HERE> ], ...
//                   insizes=[1], outsizes=[1], ...
//                   intypes=[ORTD.DATATYPE_FLOAT], outtypes=[ORTD.DATATYPE_FLOAT]  );
//
//  [sim,blk] = libdyn_conn_equation(sim, blk, list(in) );
//  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
//endfunction

// if you like to put strings as arguments for the block you could used ipar=[ length(str), ascii(str) ] for example and use "irpar_getstr" within C-Code for decoding (see irpar.h and irpar.c)


// Interfacing functions are placed in this place

function [sim, out] = ld_udp_main_receiver(sim, events, udpport, identstr, socket_fname, vecsize) // PARSEDOCU_BLOCK
    // udp main receiver - block
    //
    // This is a simulation-synchronising Block
    // 
    // EXPERIMENTAL FIXME: REMOVE
    // 

    datatype = ORTD.DATATYPE_FLOAT;

    btype = 39001;
    [sim,blk] = libdyn_new_block(sim, events, btype, ipar=[ udpport, vecsize, datatype, length(socket_fname), ascii(socket_fname), length(identstr), ascii(identstr) ], rpar=[  ], ...
    insizes=[], outsizes=[vecsize], ...
    intypes=[], outtypes=[ORTD.DATATYPE_FLOAT]  );

    //[sim,blk] = libdyn_conn_equation(sim, blk, list(in) );
    [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction




function [sim] = ld_UDPSocket_shObj(sim, events, ObjectIdentifyer, Visibility, hostname, UDPPort) // PARSEDOCU_BLOCK
    // 
    // Set-up an UDP-Socket
    //
    // hostname - Network interface to bind socket to???
    // UDPPort - UDP port to bind. If -1 then no UDP server is set-up
    // 
    // EXPERIMENTAL
    // 

    // add a postfix that identifies the type of the shared object
    ObjectIdentifyer = ObjectIdentifyer + ".UDPSocket_ShObj";



    // pack all parameters into a structure "parlist"
    parlist = new_irparam_set();

    parlist = new_irparam_elemet_ivec(parlist, UDPPort, 10); // id = 10
    parlist = new_irparam_elemet_ivec(parlist, ascii(hostname), 11); // id = 11; A string parameter

    p = combine_irparam(parlist); // convert to two vectors of integers and floating point values respectively

    // Set-up the block parameters. There are no I/O ports
    Uipar = [ p.ipar ];
    Urpar = [ p.rpar ];
    btype = 39001 + 0; // Reference to the block's type (computational function). Use the same id you are giving via the "libdyn_compfnlist_add" C-function

    [sim] =  libdyn_CreateSharedObjBlk(sim, btype, ObjectIdentifyer, Visibility, Uipar, Urpar); 
endfunction

function [sim] = ld_UDPSocket_Send(sim, events, ObjectIdentifyer, in, insize, intype) // PARSEDOCU_BLOCK
    // 
    // UDP - Send block
    //
    // in *, ORTD.DATATYPE_BINARY - input
    // 
    // EXPERIMENTAL, About to be removed
    // 

    // add a postfix that identifies the type of the shared object
    ObjectIdentifyer = ObjectIdentifyer + ".UDPSocket_ShObj";


    // pack all parameters into a structure "parlist"
    parlist = new_irparam_set();

    parlist = new_irparam_elemet_ivec(parlist, insize, 10); // id = 10
    parlist = new_irparam_elemet_ivec(parlist, intype, 11); // id = 11

    p = combine_irparam(parlist); // convert to two vectors of integers and floating point values respectively

    // Set-up the block parameters and I/O ports
    Uipar = [ p.ipar ];
    Urpar = [ p.rpar ];
    btype = 39001 + 1; // Reference to the block's type (computational function). Use the same id you are giving via the "libdyn_compfnlist_add" C-function

    insizes=[insize]; // Input port sizes
    outsizes=[]; // Output port sizes
    dfeed=[1];  // for each output 0 (no df) or 1 (a direct feedthrough to one of the inputs)
    intypes=[intype]; // datatype for each input port
    outtypes=[]; // datatype for each output port

    blocktype = 1; // 1-BLOCKTYPE_DYNAMIC (if block uses states), 2-BLOCKTYPE_STATIC (if there is only a static relationship between in- and output)

    // Create the block
    [sim, blk] = libdyn_CreateBlockAutoConfig(sim, events, btype, blocktype, Uipar, Urpar, insizes, outsizes, intypes, outtypes, dfeed, ObjectIdentifyer);

    // connect the inputs
    [sim,blk] = libdyn_conn_equation(sim, blk, list(in) ); // connect in1 to port 0 and in2 to port 1

    //   // connect the ouputs
    //  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction


function [sim, out, SrcAddr] = ld_UDPSocket_Recv(sim, events, ObjectIdentifyer, outsize) // PARSEDOCU_BLOCK
    // 
    // UDP - receiver block
    //
    // out *, ORTD.DATATYPE_BINARY - output
    // SrcAddr - information about where the package comes from (not implemented)
    // 
    // This is a simulation-synchronising Block. Everytime an UDP-Packet is received,
    // the simulation that contains this blocks goes on for one step.
    // 
    // EXPERIMENTAL
    // 

    printf("Synchronising simulation to UDP-Receiver\n");

    // add a postfix that identifies the type of the shared object
    ObjectIdentifyer = ObjectIdentifyer + ".UDPSocket_ShObj";

    //
    outtype = ORTD.DATATYPE_BINARY;

    // IPv4
    AddrSize = 4+2; // IPnumber + port

    // pack all parameters into a structure "parlist"
    parlist = new_irparam_set();

    parlist = new_irparam_elemet_ivec(parlist, outsize, 10); // id = 10
    parlist = new_irparam_elemet_ivec(parlist, outtype, 11); // id = 11

    p = combine_irparam(parlist); // convert to two vectors of integers and floating point values respectively

    // Set-up the block parameters and I/O ports
    Uipar = [ p.ipar ];
    Urpar = [ p.rpar ];
    btype = 39001 + 2; // Reference to the block's type (computational function). Use the same id you are giving via the "libdyn_compfnlist_add" C-function

    insizes=[]; // Input port sizes
    outsizes=[outsize, AddrSize]; // Output port sizes
    dfeed=[1];  // for each output 0 (no df) or 1 (a direct feedthrough to one of the inputs)
    intypes=[]; // datatype for each input port
    outtypes=[outtype, ORTD.DATATYPE_BINARY]; // datatype for each output port

    blocktype = 1; // 1-BLOCKTYPE_DYNAMIC (if block uses states), 2-BLOCKTYPE_STATIC (if there is only a static relationship between in- and output)

    // Create the block
    [sim, blk] = libdyn_CreateBlockAutoConfig(sim, events, btype, blocktype, Uipar, Urpar, insizes, outsizes, intypes, outtypes, dfeed, ObjectIdentifyer);

    //   // connect the inputs
    //  [sim,blk] = libdyn_conn_equation(sim, blk, list(in) ); // connect in1 to port 0 and in2 to port 1

    // connect the ouputs
    [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
    [sim,SrcAddr] = libdyn_new_oport_hint(sim, blk, 1);   // 1th port
endfunction

function [sim] = ld_UDPSocket_SendTo(sim, events, SendSize, ObjectIdentifyer, hostname, UDPPort, in, insize) // PARSEDOCU_BLOCK
    // 
    // UDP - Send block
    //
    // in *, ORTD.DATATYPE_BINARY - input
    // SendSize *. ORTD.DATATYPE_INT32 - Number of bytes to send
    // 
    // EXPERIMENTAL
    // 


    // add a postfix that identifies the type of the shared object
    ObjectIdentifyer = ObjectIdentifyer + ".UDPSocket_ShObj";

    // only send binary data
    intype=ORTD.DATATYPE_BINARY;


    // pack all parameters into a structure "parlist"
    parlist = new_irparam_set();

    parlist = new_irparam_elemet_ivec(parlist, insize, 10); // id = 10
    parlist = new_irparam_elemet_ivec(parlist, intype, 11); // id = 11

    parlist = new_irparam_elemet_ivec(parlist, UDPPort, 12); // id = 10
    parlist = new_irparam_elemet_ivec(parlist, ascii(hostname), 13); // id = 11; A string parameter


    p = combine_irparam(parlist); // convert to two vectors of integers and floating point values respectively

    // Set-up the block parameters and I/O ports
    Uipar = [ p.ipar ];
    Urpar = [ p.rpar ];
    btype = 39001 + 3; // Reference to the block's type (computational function). Use the same id you are giving via the "libdyn_compfnlist_add" C-function

    insizes=[insize, 1]; // Input port sizes
    outsizes=[]; // Output port sizes
    dfeed=[1];  // for each output 0 (no df) or 1 (a direct feedthrough to one of the inputs)
    intypes=[intype, ORTD.DATATYPE_INT32  ]; // datatype for each input port
    outtypes=[]; // datatype for each output port

    blocktype = 1; // 1-BLOCKTYPE_DYNAMIC (if block uses states), 2-BLOCKTYPE_STATIC (if there is only a static relationship between in- and output)

    // Create the block
    [sim, blk] = libdyn_CreateBlockAutoConfig(sim, events, btype, blocktype, Uipar, Urpar, insizes, outsizes, intypes, outtypes, dfeed, ObjectIdentifyer);

    // connect the inputs
    [sim,blk] = libdyn_conn_equation(sim, blk, list(in, SendSize) ); // connect in1 to port 0 and in2 to port 1

    //   // connect the ouputs
    //  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction






function [sim] = ld_UDPSocket_Reply(sim, events, SendSize, ObjectIdentifyer, DestAddr, in, insize) // PARSEDOCU_BLOCK
    // 
    // UDP - Send block
    //
    // in *, ORTD.DATATYPE_BINARY - input
    // SendSize *. ORTD.DATATYPE_INT32 - Number of bytes to send
    // DestAddr - dynamic representation for the destination address
    // 
    // EXPERIMENTAL not implemented by now
    // 


    // add a postfix that identifies the type of the shared object
    ObjectIdentifyer = ObjectIdentifyer + ".UDPSocket_ShObj";

    // only send binary data
    intype=ORTD.DATATYPE_BINARY;

    // IPv4
    AddrSize=4+2;

    // pack all parameters into a structure "parlist"
    parlist = new_irparam_set();

    parlist = new_irparam_elemet_ivec(parlist, insize, 10); // id = 10
    parlist = new_irparam_elemet_ivec(parlist, intype, 11); // id = 11

    //    parlist = new_irparam_elemet_ivec(parlist, UDPPort, 12); // id = 10
    //    parlist = new_irparam_elemet_ivec(parlist, ascii(hostname), 13); // id = 11; A string parameter


    p = combine_irparam(parlist); // convert to two vectors of integers and floating point values respectively

    // Set-up the block parameters and I/O ports
    Uipar = [ p.ipar ];
    Urpar = [ p.rpar ];
    btype = 39001 + 4; // Reference to the block's type (computational function). Use the same id you are giving via the "libdyn_compfnlist_add" C-function

    insizes=[insize, 1, AddrSize]; // Input port sizes
    outsizes=[]; // Output port sizes
    dfeed=[1];  // for each output 0 (no df) or 1 (a direct feedthrough to one of the inputs)
    intypes=[intype, ORTD.DATATYPE_INT32, ORTD.DATATYPE_BINARY  ]; // datatype for each input port
    outtypes=[]; // datatype for each output port

    blocktype = 1; // 1-BLOCKTYPE_DYNAMIC (if block uses states), 2-BLOCKTYPE_STATIC (if there is only a static relationship between in- and output)

    // Create the block
    [sim, blk] = libdyn_CreateBlockAutoConfig(sim, events, btype, blocktype, Uipar, Urpar, insizes, outsizes, intypes, outtypes, dfeed, ObjectIdentifyer);

    // connect the inputs
    [sim,blk] = libdyn_conn_equation(sim, blk, list(in, SendSize, DestAddr) ); // connect in1 to port 0 and in2 to port 1

    //   // connect the ouputs
    //  [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction












function [sim, out, NBytes] = ld_ConcateData(sim, events, inlist, insizes, intypes) // PARSEDOCU_BLOCK
    // 
    // Concate Data - block
    //
    // concatenates the binary representation of all inputs
    // 
    // The output is of type ORTD.DATATYPE_BINARY
    // 
    // EXPERIMENTAL
    // 


    // pack all parameters into a structure "parlist"
    //    parlist = new_irparam_set();
    // 
    //    parlist = new_irparam_elemet_ivec(parlist, insize, 10); // id = 10
    //    parlist = new_irparam_elemet_ivec(parlist, intype, 11); // id = 11
    // 
    //    p = combine_irparam(parlist); // convert to two vectors of integers and floating point values respectively

    // Set-up the block parameters and I/O ports
    Uipar = [   ];
    Urpar = [   ];
    btype = 39001 + 10; // Reference to the block's type (computational function). Use the same id you are giving via the "libdyn_compfnlist_add" C-function

    // count the number of bytes
    NBytes = 0;
    for i = 1:length(inlist)
        NBytes = NBytes + insizes(i) * libdyn_datatype_len( intypes(i) );
    end



    //   insizes=[insizes]; // Input port sizes
    outsizes=[ NBytes ]; // Output port sizes
    dfeed=[1];  // for each output 0 (no df) or 1 (a direct feedthrough to one of the inputs)
    //   intypes=[intypes]; // datatype for each input port
    outtypes=[ ORTD.DATATYPE_BINARY  ]; // datatype for each output port

    // disp(outsizes);
    // disp(outtypes);

    blocktype = 2; // 1-BLOCKTYPE_DYNAMIC (if block uses states), 2-BLOCKTYPE_STATIC (if there is only a static relationship between in- and output)

    // Create the block
    [sim, blk] = libdyn_CreateBlockAutoConfig(sim, events, btype, blocktype, Uipar, Urpar, insizes, outsizes, intypes, outtypes, dfeed);

    // connect the inputs
    //   for i = 1:length(inlist)    
    [sim,blk] = libdyn_conn_equation(sim, blk, inlist ); // connect in1 to port 0 and in2 to port 1
    //   end

    //   // connect the ouputs
    [sim,out] = libdyn_new_oport_hint(sim, blk, 0);   // 0th port
endfunction


function [sim, outlist] = ld_DisassembleData(sim, events, in, outsizes, outtypes) // PARSEDOCU_BLOCK
    // 
    // disasseble Data - block
    //
    // disassemble the binary representation of the input, which is of type ORTD.DATATYPE_BINARY
    // 
    // EXPERIMENTAL
    // 


    // pack all parameters into a structure "parlist"
    //    parlist = new_irparam_set();
    // 
    //    parlist = new_irparam_elemet_ivec(parlist, insize, 10); // id = 10
    //    parlist = new_irparam_elemet_ivec(parlist, intype, 11); // id = 11
    // 
    //    p = combine_irparam(parlist); // convert to two vectors of integers and floating point values respectively

    // Set-up the block parameters and I/O ports
    Uipar = [   ];
    Urpar = [   ];
    btype = 39001 + 11; // Reference to the block's type (computational function). Use the same id you are giving via the "libdyn_compfnlist_add" C-function

    // count the number of bytes
    NBytes = 0;
    for i = 1:length(outsizes)
        if outsizes(i) < 1
          error("ld_DisassembleData: Output port %d has size < 1", outsizes(i))    
        end
        
        NBytes = NBytes + outsizes(i)*libdyn_datatype_len( outtypes(i) );
    end



    //   insizes=[insizes]; // Input port sizes
    insizes=[ NBytes ]; // Output port sizes
    dfeed=[1];  // for each output 0 (no df) or 1 (a direct feedthrough to one of the inputs)
    //   intypes=[intypes]; // datatype for each input port
    intypes=[ ORTD.DATATYPE_BINARY  ]; // datatype for each output port

    // disp(outsizes);
    // disp(outtypes);

    blocktype = 2; // 1-BLOCKTYPE_DYNAMIC (if block uses states), 2-BLOCKTYPE_STATIC (if there is only a static relationship between in- and output)

    // Create the block
    [sim, blk] = libdyn_CreateBlockAutoConfig(sim, events, btype, blocktype, Uipar, Urpar, insizes, outsizes, intypes, outtypes, dfeed);

    // connect the inputs
    //   for i = 1:length(inlist)    
    [sim,blk] = libdyn_conn_equation(sim, blk, list(in) ); // connect in1 to port 0 and in2 to port 1
    //   end

    //   // connect the ouputs
    outlist = list();
    for i = 1:length(outtypes)
        [sim,outlist($+1)] = libdyn_new_oport_hint(sim, blk, i-1);   // 0th port
    end
endfunction




function [sim, outlist, Success] = ld_DisassembleData2(sim, events, in, inBytes, MaxinBytes, ByteOfs, outsizes, outtypes) // PARSEDOCU_BLOCK
    // 
    // disasseble Data - block
    //
    // disassemble the binary representation of the input, which is of type ORTD.DATATYPE_BINARY
    // 
    // 
    // in *(BINARY) - binary input data
    // inBytes * (INT32) - number of valid input bytes
    // MaxinBytes - Number of maximal input bytes (determines the size of in)
    // ByteOfs * (INT32) - ofset at which the disassembly takes place (index starts at 0)
    // outsizes - array of output sizes
    // insizes - array of output types
    //
    // Success (INT32) - if the Disassebly could successfully be performed
    // outlist - *list() of the decomposed data
    // 


    // pack all parameters into a structure "parlist"
    //    parlist = new_irparam_set();
    // 
    //    parlist = new_irparam_elemet_ivec(parlist, insize, 10); // id = 10
    //    parlist = new_irparam_elemet_ivec(parlist, intype, 11); // id = 11
    // 
    //    p = combine_irparam(parlist); // convert to two vectors of integers and floating point values respectively

    // Set-up the block parameters and I/O ports
    Uipar = [ MaxinBytes  ];
    Urpar = [   ];
    btype = 39001 + 12; // Reference to the block's type (computational function). Use the same id you are giving via the "libdyn_compfnlist_add" C-function

    // count the number of bytes
    NBytes = 0;
    outsizes__ = list();
    outtype__ = list();
    
//    outtypes__(1) = ORTD.DATATYPE_INT32;
//    outsizes__(1) = 1;
    
    for i = 1:length(outsizes)
        if outsizes(i) < 1
          error("ld_DisassembleData: Output port %d has size < 1", outsizes(i))    
        end
        
        NBytes = NBytes + outsizes(i)*libdyn_datatype_len( outtypes(i) );
        
//        // copy
//        outsizes__(i+1) = outsizes(i);
//        outtypes__(i+1) = outtypes(i);
    end
    
    outsizes__ = [1, outsizes(:)' ];
    outtypes__ = [ORTD.DATATYPE_INT32, outtypes(:)' ];



    //   insizes=[insizes]; // Input port sizes
    insizes=[ MaxinBytes, 1, 1 ]; // Output port sizes
    dfeed=[1];  // for each output 0 (no df) or 1 (a direct feedthrough to one of the inputs)
    //   intypes=[intypes]; // datatype for each input port
    intypes=[ ORTD.DATATYPE_BINARY, ORTD.DATATYPE_INT32, ORTD.DATATYPE_INT32  ]; // datatype for each output port

    // disp(outsizes);
    // disp(outtypes);

    blocktype = 2; // 1-BLOCKTYPE_DYNAMIC (if block uses states), 2-BLOCKTYPE_STATIC (if there is only a static relationship between in- and output)

    // Create the block
    [sim, blk] = libdyn_CreateBlockAutoConfig(sim, events, btype, blocktype, Uipar, Urpar, insizes, outsizes__, intypes, outtypes__, dfeed);

    // connect the inputs
    //   for i = 1:length(inlist)    
    [sim,blk] = libdyn_conn_equation(sim, blk, list(in, inBytes, ByteOfs ) ); // connect in1 to port 0 and in2 to port 1
    //   end

    //   // connect the ouputs
    [sim, Success] = libdyn_new_oport_hint(sim, blk, 0 ); 
    
    outlist = list();
    for i = 1:length(outtypes)
        [sim,outlist($+1)] = libdyn_new_oport_hint(sim, blk, i );   // 
    end
endfunction








// 
// 
//   A packet based communication interface from ORTD using UDP datagrams to e.g.
//   nodejs. 
//   webappUDP.js is the counterpart that provides a web-interface 
// 
// Current Rev: 9
// 
// Revisions:
// 
// 27.3.14 - possibility to reservate sources
// 3.4.14  - small re-arrangements
// 4.4.14  - Bugfixes
// 7.4.14  - Bugfix
// 12.6.14 - Bugfix
// 2.11.14 - Added group finalising packet
// rev 9 19.3.14 - Added ability to send GUI-configurations to PapI 
// 


function ORTD_print_json(a)
  jsonstr = struct2json(a);
  disp(jsonstr);
endfunction

    function jsonstr = struct2json(a)
    //
    // Copyright 2015 Christian Klauer, klauer@control.tu-berlin.de, Licensed under BSD-License
    //
    // Rev 1 as of 4.3.15: Initial version
    // Rev 2 as of 4.3.15: added arrays of strings that are defined by e.g. list('str1', 'str2')
    // Rev 3 as of 17.3.15: added support for vectors
    // Rev 4 as of 6.4.15: support for general list() structures and matrices; optimized the code 
    //
    // Example usage:
    //
    //    clear a;
    //    a.Field1 = 2;
    //    a.Field2 = "jkh";
    //    a.Field3 = 1.2;
    //    a.F3.name = "Joe";
    //    a.F3.age = 32;
    //    a.strArray =  list( 'V1', 'Xn' );
    //    a.vector = [1,2,3,4.5535894379345];
    //    a.Matrix = diag([1,2,3]);
    //
    //    struc.e = 2;
    //    struc.f = 3;
    //    a.list = list( 1, "Test", struc );
    //
    //    jsonstr = struct2json(a);
    //    disp(jsonstr);
    //    
    //     Warning: For strings make sure you escape the special characters that are used by the JSON-format!
    // 
    // The precision of float values can be adjusted by previously using the command "format".
    //

    function valstr=val2str(val)
        select typeof(val)

        case "string" // export a string
            if length(length(val)) == 1 then
                valstr = """" + string(val) + """";
            end

        case "list" // convert Scilab list()s to arrays
            valstr = '[';

            N = length(val);
            for i=1:(N-1)
                valstr = valstr + val2str( val(i) ) + ',';
            end
            valstr = valstr +  val2str( val(N) )   + ']';

        case "constant" // export numerical values in form of vectors, matrices or single values
            [n,m] = size(val);

            if n == 1 & m == 1 then // single value
                if isnan(val) then
                    valstr = '""NaN""';
                else                
                    valstr = string(val);
                end
            else
                if n == 1 then // row vector
                    valstr = sci2exp(val(:)');
                end
            end

            if n > 1 then // a matrix or multiple columns
                valstr = '[';
                // for all lines
                for i=1:(n-1)
                    mline = val(i,:);
                    valstr = valstr + sci2exp(mline) + ',';
                end                  
                mline = val(n,:);
                valstr = valstr + sci2exp(mline) ;                  
                valstr = valstr + ']';
            end

        case "st" // export a structure
            a=val;
            F = fieldnames(a);

            valstr = "{";
            N = length(length(F));

            for i = 1:(N-1) // iterate through all fields
                f = F(i);
                val = eval('a.'+f); // extract an element of the struct
                valstrIT = val2str(val);
                valstr = valstr + """" + f + """" + ':' + valstrIT + ',';
            end

            f = F(N);
            val = eval('a.'+f);
            valstrIT = val2str(val);
            valstr = valstr + """" + f + """" + ':' + valstrIT + '}';

        else
            valstr = """" + "Datatype " + typeof(val) + " not supported" + """";
        end

    endfunction

    jsonstr=val2str(a);
endfunction


function [PacketFramework, PluginUname] = ld_PF_addplugin(PacketFramework, PluginType, PluginName, PluginSize, PluginPos)
    // inc counter
    PacketFramework.PluginID_counter = PacketFramework.PluginID_counter + 1;

    PluginUname = "Plugin" + string(PacketFramework.PluginID_counter);


    // Add new plugin to the struct
    PacketFramework.PaPIConfig.ToCreate(PluginUname).identifier.value =  PluginType;

    PacketFramework.PaPIConfig.ToCreate(PluginUname).config.name.value = PluginName;
    PacketFramework.PaPIConfig.ToCreate(PluginUname).config.size.value = PluginSize;
    PacketFramework.PaPIConfig.ToCreate(PluginUname).config.position.value = PluginPos;

endfunction

function [PacketFramework, PluginUname] = ld_PF_addpluginToTab(PacketFramework, PluginType, PluginName, PluginSize, PluginPos, TabName)
    // inc counter
    PacketFramework.PluginID_counter = PacketFramework.PluginID_counter + 1;

    PluginUname = "Plugin" + string(PacketFramework.PluginID_counter);


    // Add new plugin to the struct
    PacketFramework.PaPIConfig.ToCreate(PluginUname).identifier.value =  PluginType;

    PacketFramework.PaPIConfig.ToCreate(PluginUname).config.name.value = PluginName;
    PacketFramework.PaPIConfig.ToCreate(PluginUname).config.size.value = PluginSize;
    PacketFramework.PaPIConfig.ToCreate(PluginUname).config.position.value = PluginPos;
    PacketFramework.PaPIConfig.ToCreate(PluginUname).config.tab.value = TabName;

endfunction

function [PacketFramework] = ld_PF_changePluginSetting(PacketFramework, PluginUname, PluginSetting, PluginSettingValue)
    PacketFramework.PaPIConfig.ToCreate(PluginUname).config(PluginSetting).value = PluginSettingValue;
endfunction

function [PacketFramework] = ld_PF_changePluginConfig(PacketFramework, PluginUname, PluginSettingsList)
    for iSetting=PluginSettingsList
        PacketFramework = ld_PF_changePluginSetting(PacketFramework, PluginUname, iSetting(1), iSetting(2));
    end
endfunction

function [PacketFramework, PluginUname] = ld_PF_addpluginAdvanced(PacketFramework, PluginType, PluginName, PluginSize, PluginPos, TabName, PluginSettingsList)
    // inc counter
    PacketFramework.PluginID_counter = PacketFramework.PluginID_counter + 1;

    PluginUname = "Plugin" + string(PacketFramework.PluginID_counter);


    // Add new plugin to the struct
    PacketFramework.PaPIConfig.ToCreate(PluginUname).identifier.value =  PluginType;

    PacketFramework.PaPIConfig.ToCreate(PluginUname).config.name.value = PluginName;
    PacketFramework.PaPIConfig.ToCreate(PluginUname).config.size.value = PluginSize;
    PacketFramework.PaPIConfig.ToCreate(PluginUname).config.position.value = PluginPos;
    PacketFramework.PaPIConfig.ToCreate(PluginUname).config.tab.value = TabName;

    PacketFramework = ld_PF_changePluginConfig(PacketFramework, PluginUname, PluginSettingsList);

endfunction

function [PacketFramework] = ld_PF_addpluginAdvancedInclControl(PacketFramework, PluginType, PluginName, PluginSize, PluginPos, TabName, PluginSettingsList, ControlParameterID, ControlBlock)
    // inc counter
    PacketFramework.PluginID_counter = PacketFramework.PluginID_counter + 1;

    PluginUname = "Plugin" + string(PacketFramework.PluginID_counter);


    // Add new plugin to the struct
    PacketFramework.PaPIConfig.ToCreate(PluginUname).identifier.value =  PluginType;

    PacketFramework.PaPIConfig.ToCreate(PluginUname).config.name.value = PluginName;
    PacketFramework.PaPIConfig.ToCreate(PluginUname).config.size.value = PluginSize;
    PacketFramework.PaPIConfig.ToCreate(PluginUname).config.position.value = PluginPos;
    PacketFramework.PaPIConfig.ToCreate(PluginUname).config.tab.value = TabName;

    PacketFramework = ld_PF_changePluginConfig(PacketFramework, PluginUname, PluginSettingsList);

    PacketFramework = ld_PF_addcontrolbyID(PacketFramework, PluginUname, ControlBlock, ControlParameterID);

endfunction

function [PacketFramework] = ld_PF_addpluginAdvancedInclSub(PacketFramework, PluginType, PluginName, PluginSize, PluginPos, TabName, PluginSettingsList, SubSourceIDs, SubBlock)
    // inc counter
    PacketFramework.PluginID_counter = PacketFramework.PluginID_counter + 1;

    PluginUname = "Plugin" + string(PacketFramework.PluginID_counter);


    // Add new plugin to the struct
    PacketFramework.PaPIConfig.ToCreate(PluginUname).identifier.value =  PluginType;

    PacketFramework.PaPIConfig.ToCreate(PluginUname).config.name.value = PluginName;
    PacketFramework.PaPIConfig.ToCreate(PluginUname).config.size.value = PluginSize;
    PacketFramework.PaPIConfig.ToCreate(PluginUname).config.position.value = PluginPos;
    PacketFramework.PaPIConfig.ToCreate(PluginUname).config.tab.value = TabName;

    PacketFramework = ld_PF_changePluginConfig(PacketFramework, PluginUname, PluginSettingsList);
    
    PacketFramework = ld_PF_addsubsbyID(PacketFramework, PluginUname, SubBlock, SubSourceIDs);

endfunction

function [PacketFramework] = ld_PF_addsubs(PacketFramework, SubPluginUname, SubBlock, SubSignals)
    if (isfield(PacketFramework.PaPIConfig, "ToSub"))
        if (isfield(PacketFramework.PaPIConfig.ToSub, SubPluginUname))
            PacketFramework.PaPIConfig.ToSub(SubPluginUname).signals = lstcat(PacketFramework.PaPIConfig.ToSub(SubPluginUname).signals, SubSignals);
        else
            PacketFramework.PaPIConfig.ToSub(SubPluginUname).signals =  SubSignals;
        end
    else
        PacketFramework.PaPIConfig.ToSub(SubPluginUname).signals =  SubSignals;
    end
    PacketFramework.PaPIConfig.ToSub(SubPluginUname).block =  SubBlock;
endfunction

function [PacketFramework] = ld_PF_addsubsbyID(PacketFramework, SubPluginUname, SubBlock, SubSignalIDs)
    SubSignals = list();
    for i=SubSignalIDs;
        SubSignals($+1) = PacketFramework.Sources(i+1).SourceName;
    end;
    PacketFramework = ld_PF_addsubs(PacketFramework, SubPluginUname, SubBlock, SubSignals);
endfunction

function [PacketFramework] = ld_PF_addcontrol(PacketFramework, ControlPluginUname, ControlBlock, ControlParam)
    PacketFramework.PaPIConfig.ToControl(ControlPluginUname).block =  ControlBlock;
    PacketFramework.PaPIConfig.ToControl(ControlPluginUname).parameter =  ControlParam;
endfunction

function [PacketFramework] = ld_PF_addcontrolbyID(PacketFramework, ControlPluginUname, ControlBlock, ControlParamID)
    PacketFramework = ld_PF_addcontrol(PacketFramework, ControlPluginUname, ControlBlock, PacketFramework.Parameters(ControlParamID+1).ParameterName);
endfunction

function [PacketFramework,SourceID] = ld_PF_addsource(PacketFramework, NValues_send, datatype, SourceName, Demux)
    SourceID = PacketFramework.SourceID_counter;

    Source.SourceName = SourceName;
    Source.SourceID = SourceID;
    Source.NValues_send = NValues_send;
    Source.datatype =  datatype;
    if typeof(Demux) == 'list' then
      Source.Demux = Demux;
    end

    // Add new source to the list
    PacketFramework.Sources($+1) = Source;

    // inc counter
    PacketFramework.SourceID_counter = PacketFramework.SourceID_counter + 1;
endfunction

function [PacketFramework,SourceID] = ld_PF_addsourceInclSub(PacketFramework, NValues_send, datatype, SourceName, SubPluginUname, SubBlock)
    [PacketFramework,SourceID] = ld_PF_addsource(PacketFramework, NValues_send, datatype, SourceName, 0 );
    PacketFramework = ld_PF_addsubsbyID(PacketFramework, SubPluginUname, SubBlock, SourceID);
endfunction

function [PacketFramework,ParameterID,MemoryOfs] = ld_PF_addparameter(PacketFramework, NValues, datatype, ParameterName, optionalInitValue)
    if ~exists('optionalInitValue', 'local')
        optionalInitValue = zeros(1,NValues);
    end
    if (length(optionalInitValue) ~= NValues)
        error("length(optionalInitValue) = " + string(length(optionalInitValue)) + " ~= NValues = " + string(NValues));
    end
    ParameterID = PacketFramework.Parameterid_counter;

    Parameter.ParameterName = ParameterName;
    Parameter.ParameterID = ParameterID;
    Parameter.NValues = NValues;
    Parameter.datatype =  datatype;
    Parameter.InitialValue = optionalInitValue;
    Parameter.MemoryOfs = PacketFramework.ParameterMemOfs_counter;

    // Add new source to the list
    PacketFramework.Parameters($+1) = Parameter;

    // inc counters
    PacketFramework.Parameterid_counter = PacketFramework.Parameterid_counter + 1;
    PacketFramework.ParameterMemOfs_counter = PacketFramework.ParameterMemOfs_counter + NValues;

    // return values
    ParameterID = Parameter.ParameterID; 
    MemoryOfs = Parameter.MemoryOfs;
endfunction

function [PacketFramework,ParameterID,MemoryOfs] = ld_PF_addparameterInclControl(PacketFramework, NValues, datatype, ParameterName, ControlPluginUname, ControlBlock, optionalInitValue)
    if ~exists('optionalInitValue', 'local')
        optionalInitValue = zeros(1,NValues);
    end
    [PacketFramework,ParameterID,MemoryOfs] = ld_PF_addparameter(PacketFramework, NValues, datatype, ParameterName, optionalInitValue);
    PacketFramework = ld_PF_addcontrolbyID(PacketFramework, ControlPluginUname, ControlBlock, ParameterID);
endfunction

function [sim, PacketFramework, Parameter] = ld_PF_Parameter(sim, PacketFramework, NValues, datatype, ParameterName, optionalInitValue) // PARSEDOCU_BLOCK
    // 
    // Define a parameter
    // 
    // NValues - amount of data sets
    // datatype - only ORTD.DATATYPE_FLOAT for now
    // ParameterName - a unique string decribing the parameter
    // 
    // 
    // 
    if ~exists('optionalInitValue', 'local')
        optionalInitValue = zeros(1,NValues);
    end
    [PacketFramework,ParameterID,MemoryOfs] = ld_PF_addparameter(PacketFramework, NValues, datatype, ParameterName, optionalInitValue);

    // read data from global memory
    [sim, readI] = ld_const(sim, 0, MemoryOfs); // start at index 1
    [sim, Parameter] = ld_read_global_memory(sim, 0, index=readI, ident_str=PacketFramework.InstanceName+"Memory", ...
    datatype, NValues);
endfunction

function [sim, PacketFramework, ParameterID, Parameter] = ld_PF_ParameterForPlugin(sim, PacketFramework, NValues, datatype, ParameterName, optionalInitValue) // PARSEDOCU_BLOCK
    // 
    // Define a parameter and get the ParameterID to control it by a plugin
    // 
    // NValues - amount of data sets
    // datatype - only ORTD.DATATYPE_FLOAT for now
    // ParameterName - a unique string decribing the parameter
    // 
    // 
    // 
    if ~exists('optionalInitValue', 'local')
        optionalInitValue = zeros(1,NValues);
    end
    [PacketFramework,ParameterID,MemoryOfs] = ld_PF_addparameter(PacketFramework, NValues, datatype, ParameterName, optionalInitValue);

    // read data from global memory
    [sim, readI] = ld_const(sim, 0, MemoryOfs); // start at index 1
    [sim, Parameter] = ld_read_global_memory(sim, 0, index=readI, ident_str=PacketFramework.InstanceName+"Memory", ...
    datatype, NValues);
endfunction

function [sim, PacketFramework, Parameter] = ld_PF_ParameterInclControl(sim, PacketFramework, NValues, datatype, ParameterName, ControlPluginUname, ControlBlock, optionalInitValue) // PARSEDOCU_BLOCK
    // 
    // Define a parameter and it's control unit
    // 
    // NValues - amount of data sets
    // datatype - only ORTD.DATATYPE_FLOAT for now
    // ParameterName - a unique string decribing the parameter
    // ControlPluginUname - a unique string describing the control unit
    // ControlBlock - e.g. 'Click_Event' for a Button
    // 
    // 
    if ~exists('optionalInitValue', 'local')
        optionalInitValue = zeros(1,NValues);
    end
    [PacketFramework,ParameterID,MemoryOfs] = ld_PF_addparameterInclControl(PacketFramework, NValues, datatype, ParameterName, ControlPluginUname, ControlBlock, optionalInitValue);

    // read data from global memory
    [sim, readI] = ld_const(sim, 0, MemoryOfs); // start at index 1
    [sim, Parameter] = ld_read_global_memory(sim, 0, index=readI, ident_str=PacketFramework.InstanceName+"Memory", ...
    datatype, NValues);
endfunction


// Send a signal via UDP, a simple protocoll is defined, internal function
function [sim] = ld_PF_ISendUDP(sim, PacketFramework, Signal, NValues_send, datatype, SourceID)
    InstanceName = PacketFramework.InstanceName;
    [sim,one] = ld_const(sim, 0, 1);

    // Packet counter, so the order of the network packages can be determined
    [sim, Counter] = ld_modcounter(sim, 0, in=one, initial_count=0, mod=100000);
    [sim, Counter_int32] = ld_ceilInt32(sim, 0, Counter);

    // Source ID
    [sim, SourceID] = ld_const(sim, 0, SourceID);
    [sim, SourceID_int32] = ld_ceilInt32(sim, 0, SourceID);

    // Sender ID
    [sim, SenderID] = ld_const(sim, 0, PacketFramework.SenderID); // random number
    [sim, SenderID_int32] = ld_ceilInt32(sim, 0, SenderID);

    // make a binary structure
    [sim, Data, NBytes] = ld_ConcateData(sim, 0, ...
    inlist=list(SenderID_int32, Counter_int32, SourceID_int32, Signal ), insizes=[1,1,1,NValues_send], ...
    intypes=[ ORTD.DATATYPE_INT32, ORTD.DATATYPE_INT32, ORTD.DATATYPE_INT32, datatype ] );

    printf("The size of the UDP-packets will be %d bytes.\n", NBytes);

    // send to the network 
    [sim, NBytes__] = ld_constvecInt32(sim, 0, vec=NBytes); // the number of bytes that are actually send is dynamic, but must be smaller or equal to 
    [sim] = ld_UDPSocket_SendTo(sim, 0, SendSize=NBytes__, ObjectIdentifyer=InstanceName+"aSocket", ...
    hostname=PacketFramework.Configuration.DestHost, ...
    UDPPort=PacketFramework.Configuration.DestPort, in=Data, ...
    insize=NBytes);

endfunction


function [sim, PacketFramework] = ld_SendPacketMux(sim, PacketFramework, Signal, NValues_send, datatype, SourceName, Demux) // PARSEDOCU_BLOCK // PARSEDOCU_BLOCK
    // 
    // Stream data - block
    // 
    // Signal - the signal to stream
    // NValues_send - the vector length of Signal
    // datatype - only ORTD.DATATYPE_FLOAT by now
    // SourceName - a unique string identifier descring the stream
    // Demux - information for the receiver on how to demultiplex the packet into individual signals.
    // 
    // 

    [PacketFramework,SourceID] = ld_PF_addsource(PacketFramework, NValues_send, datatype, SourceName, Demux );
    [sim]=ld_PF_ISendUDP(sim, PacketFramework, Signal, NValues_send, datatype, SourceID);
endfunction

function [sim, PacketFramework] = ld_SendPacket(sim, PacketFramework, Signal, NValues_send, datatype, SourceName) // PARSEDOCU_BLOCK // PARSEDOCU_BLOCK
    // 
    // Stream data - block
    // 
    // Signal - the signal to stream
    // NValues_send - the vector length of Signal
    // datatype - only ORTD.DATATYPE_FLOAT by now
    // SourceName - a unique string identifier descring the stream
    // 
    // 
    // 

    [PacketFramework,SourceID] = ld_PF_addsource(PacketFramework, NValues_send, datatype, SourceName, 0 );
    [sim]=ld_PF_ISendUDP(sim, PacketFramework, Signal, NValues_send, datatype, SourceID);
endfunction

function [sim, PacketFramework, SourceID] = ld_SendPacketForPlugin(sim, PacketFramework, Signal, NValues_send, datatype, SourceName) // PARSEDOCU_BLOCK // PARSEDOCU_BLOCK
    // 
    // Stream data - block and get the SourceID to visualize it by a plugin
    // 
    // Signal - the signal to stream
    // NValues_send - the vector length of Signal
    // datatype - only ORTD.DATATYPE_FLOAT by now
    // SourceName - a unique string identifier descring the stream
    // 
    // 
    // 

    [PacketFramework,SourceID] = ld_PF_addsource(PacketFramework, NValues_send, datatype, SourceName, 0 );
    [sim]=ld_PF_ISendUDP(sim, PacketFramework, Signal, NValues_send, datatype, SourceID);
endfunction

function [sim, PacketFramework] = ld_SendPacketInclSub(sim, PacketFramework, Signal, NValues_send, datatype, SourceName, SubPluginUname, SubBlock) // PARSEDOCU_BLOCK // PARSEDOCU_BLOCK
    // 
    // Stream data - block including a subscibtion to a plugin
    // 
    // Signal - the signal to stream
    // NValues_send - the vector length of Signal
    // datatype - only ORTD.DATATYPE_FLOAT by now
    // SourceName - a unique string identifier descring the stream
    // SubPluginUname - a unique string identifier describing the plugin
    // SubBlock - e.g. 'SourceGroup0'
    // 

    [PacketFramework,SourceID] = ld_PF_addsourceInclSub(PacketFramework, NValues_send, datatype, SourceName, SubPluginUname, SubBlock)
    [sim]=ld_PF_ISendUDP(sim, PacketFramework, Signal, NValues_send, datatype, SourceID);
endfunction


function [sim, PacketFramework] = ld_PF_InitInstance(sim, InstanceName, Configuration) // PARSEDOCU_BLOCK
    // 
    // Initialise an instance of the Packet Framework
    //   
    // InstanceName - a unique string identifier for the instance
    // Configuration must include the following properties:
    // 
    //   Configuration.UnderlyingProtocoll = "UDP"
    //   Configuration.DestHost
    //   Configuration.DestPort
    //   Configuration.LocalSocketHost
    //   Configuration.LocalSocketPort
    // 
    // 
    // Example:
    // 
    // 
    //   Configuration.UnderlyingProtocoll = "UDP";
    //   Configuration.DestHost = "127.0.0.1";
    //   Configuration.DestPort = 20000;
    //   Configuration.LocalSocketHost = "127.0.0.1";
    //   Configuration.LocalSocketPort = 20001;
    //   [sim, PacketFramework] = ld_PF_InitInstance(sim, InstanceName="UDPCommunication", Configuration);
    // 
    // 
    // 
    // Also consider the file webappUDP.js as the counterpart that communicates to ORTD-simulations
    // 
    // 

    // initialise structure for sources
    PacketFramework.InstanceName = InstanceName;
    PacketFramework.Configuration = Configuration;

    PacketFramework.Configuration.debugmode = %F;

    //   disp(Configuration.UnderlyingProtocoll)

    if Configuration.UnderlyingProtocoll == 'UDP'
        null;
    else
        error("PacketFramework: Only UDP supported up to now");
    end

    // possible packet sizes for UDP
    PacketFramework.TotalElemetsPerPacket = floor((1400-3*4)/8); // number of doubles values that fit into one UDP-packet with maximal size of 1400 bytes
    PacketFramework.PacketSize = PacketFramework.TotalElemetsPerPacket*8 + 3*4;

    // sources
    PacketFramework.SourceID_counter = 0;
    PacketFramework.Sources = list();

    // parameters
    PacketFramework.Parameterid_counter = 0;
    PacketFramework.ParameterMemOfs_counter = 1; // start at the first index in the memory
    PacketFramework.Parameters = list();

    // plugins
    PacketFramework.PluginID_counter = 0;

    PacketFramework.SenderID = 1295793;

    // Open an UDP-Port in server mode
    [sim] = ld_UDPSocket_shObj(sim, 0, ObjectIdentifyer=InstanceName+"aSocket", Visibility=0, ...
    hostname=PacketFramework.Configuration.LocalSocketHost, ...
    UDPPort=PacketFramework.Configuration.LocalSocketPort);
endfunction


// Send a signal via UDP, a simple protocoll is defined, internal function
function [sim] = ld_PF_SendGroupFinshUDP(sim, PacketFramework, GroupID)
    InstanceName = PacketFramework.InstanceName;
    [sim,one] = ld_const(sim, 0, 1);

    // Packet counter, so the order of the network packages can be determined
    [sim, Counter] = ld_modcounter(sim, 0, in=one, initial_count=0, mod=100000);
    [sim, Counter_int32] = ld_ceilInt32(sim, 0, Counter);

    // Source ID
    [sim, SourceID] = ld_const(sim, 0, -1);                   // -1 means finish a group of sources
    [sim, SourceID_int32] = ld_ceilInt32(sim, 0, SourceID);

    // Group ID
    [sim, GroupID_] = ld_const(sim, 0, GroupID);                   // -1 means finish a group of sources
    [sim, GroupID_int32] = ld_ceilInt32(sim, 0, GroupID_);

    // Sender ID
    [sim, SenderID] = ld_const(sim, 0, PacketFramework.SenderID); // random number
    [sim, SenderID_int32] = ld_ceilInt32(sim, 0, SenderID);

    // make a binary structure
    [sim, Data, NBytes] = ld_ConcateData(sim, 0, ...
    inlist=list(SenderID_int32, Counter_int32, SourceID_int32, GroupID_int32 ), insizes=[1,1,1,1], ...
    intypes=[ ORTD.DATATYPE_INT32, ORTD.DATATYPE_INT32, ORTD.DATATYPE_INT32, ORTD.DATATYPE_INT32 ] );

    //  printf("The size of the UDP-packets will be %d bytes.\n", NBytes);

    // send to the network 
    [sim, NBytes__] = ld_constvecInt32(sim, 0, vec=NBytes); // the number of bytes that are actually send is dynamic, but must be smaller or equal to 
    [sim] = ld_UDPSocket_SendTo(sim, 0, SendSize=NBytes__, ObjectIdentifyer=InstanceName+"aSocket", ...
    hostname=PacketFramework.Configuration.DestHost, ...
    UDPPort=PacketFramework.Configuration.DestPort, in=Data, ...
    insize=NBytes);


    // [sim] = ld_printf(sim, ev, GroupID_, "Sent finish packet ", 1);
endfunction


// Send the newConfigAvailable Signal via UDP, a simple protocoll is defined, internal function
function [sim] = ld_PF_SendNewConfigAvailableUDP(sim, PacketFramework)
    InstanceName = PacketFramework.InstanceName;
    [sim,one] = ld_const(sim, 0, 1);

    // Packet counter, so the order of the network packages can be determined
    [sim, Counter] = ld_modcounter(sim, 0, in=one, initial_count=0, mod=100000);
    [sim, Counter_int32] = ld_ceilInt32(sim, 0, Counter);

    // Source ID
    [sim, SourceID] = ld_const(sim, 0, -2);                   // -2 means a new config is available
    [sim, SourceID_int32] = ld_ceilInt32(sim, 0, SourceID);

    // Group ID
    [sim, GroupID_] = ld_const(sim, 0, 0);                   // 0 (not used)
    [sim, GroupID_int32] = ld_ceilInt32(sim, 0, GroupID_);

    // Sender ID
    [sim, SenderID] = ld_const(sim, 0, PacketFramework.SenderID); // random number
    [sim, SenderID_int32] = ld_ceilInt32(sim, 0, SenderID);

    // make a binary structure
    [sim, Data, NBytes] = ld_ConcateData(sim, 0, ...
    inlist=list(SenderID_int32, Counter_int32, SourceID_int32, GroupID_int32 ), insizes=[1,1,1,1], ...
    intypes=[ ORTD.DATATYPE_INT32, ORTD.DATATYPE_INT32, ORTD.DATATYPE_INT32, ORTD.DATATYPE_INT32 ] );

    //  printf("The size of the UDP-packets will be %d bytes.\n", NBytes);

    // send to the network 
    [sim, NBytes__] = ld_constvecInt32(sim, 0, vec=NBytes); // the number of bytes that are actually send is dynamic, but must be smaller or equal to 
    [sim] = ld_UDPSocket_SendTo(sim, 0, SendSize=NBytes__, ObjectIdentifyer=InstanceName+"aSocket", ...
    hostname=PacketFramework.Configuration.DestHost, ...
    UDPPort=PacketFramework.Configuration.DestPort, in=Data, ...
    insize=NBytes);


    // [sim] = ld_printf(sim, ev, GroupID_, "Sent finish packet ", 1);
endfunction

// Send the configuration via UDP, a simple protocoll is defined, internal function
function [sim] = ld_PF_SendConfigUDP(sim, PacketFramework)
    InstanceName = PacketFramework.InstanceName;

    str_PF_Export = ld_PF_Export_str(PacketFramework);
    //  fd = mopen("ExportStr.json",'wt');
    //  mfprintf(fd,'%s', str_PF_Export_);
    //  mclose(fd);
    //  str_PF_Export2 = strsubst(str_PF_Export1, char(10), "");
    //  str_PF_Export3 = strsubst(str_PF_Export2, " ", "");
    //  str_PF_Export_ = strsubst(str_PF_Export3, "$", " ");
    //  str_PF_Export = " " + str_PF_Export_;
    //  fd = mopen("testExportStr.json",'wt');
    //  mfprintf(fd,'%s', str_PF_Export);
    //  mclose(fd);
    strLength = length(str_PF_Export);
    maxPacketLength = 1400;                  // Max payload size for an UDP packet
    maxPacketStrLength = maxPacketLength - 4*4;
    nPackets = ceil(strLength/maxPacketStrLength);

    // Source ID
    [sim, SourceID] = ld_const(sim, 0, -4);                   // -4 means configItem
    [sim, SourceID_int32] = ld_ceilInt32(sim, 0, SourceID);

    // nPackets
    [sim, nPackets_] = ld_const(sim, 0, nPackets);                   // the number of config packets for the whole configuration
    [sim, nPackets_int32] = ld_ceilInt32(sim, 0, nPackets_);

    // Sender ID
    [sim, SenderID] = ld_const(sim, 0, PacketFramework.SenderID); // random number
    [sim, SenderID_int32] = ld_ceilInt32(sim, 0, SenderID);

    // for each sub-packet generate blocks that send the parts to PaPi
    for i=1:nPackets
        // Packet counter, so the order of the network packages can be determined
        [sim, Counter] = ld_const(sim, ev, i);
        [sim, Counter_int32] = ld_ceilInt32(sim, 0, Counter);
        partBegin = (i-1)*maxPacketStrLength+1;
        partEnd = i*maxPacketStrLength;
        if (partEnd > strLength)
            partEnd = strLength;
        end
        strPart_PF_Export = part(str_PF_Export, partBegin:partEnd);
        sendSize = length(strPart_PF_Export);
        [sim, partPF_Export_bin] = ld_const_bin(sim, ev, ascii(strPart_PF_Export));

        // make a binary structure
        [sim, Data, NBytes] = ld_ConcateData(sim, 0, ...
        inlist=list(SenderID_int32, Counter_int32, SourceID_int32, nPackets_int32, partPF_Export_bin), insizes=[1,1,1,1,sendSize], ...
        intypes=[ ORTD.DATATYPE_INT32, ORTD.DATATYPE_INT32, ORTD.DATATYPE_INT32, ORTD.DATATYPE_INT32, ORTD.DATATYPE_BINARY ] );


        if PacketFramework.Configuration.debugmode then         
            printf("The size of the UDP-packets will be %d bytes.\n", NBytes);

            [sim] = ld_printf(sim, ev, SenderID, "SenderID", 1);
            [sim] = ld_printf(sim, ev, SourceID, "Sent config packet number " + string(i) + " ", 1);
            [sim] = ld_printf(sim, ev, Counter, "Packet Count ", 1);
            [sim] = ld_printf(sim, ev, nPackets_, "Number of Packet ", 1);
        end

        // send to the network 
        [sim, NBytes__] = ld_constvecInt32(sim, 0, vec=NBytes); // the number of bytes that are actually send is dynamic, but must be smaller or equal to 
        [sim] = ld_UDPSocket_SendTo(sim, 0, SendSize=NBytes__, ObjectIdentifyer=InstanceName+"aSocket", ...
        hostname=PacketFramework.Configuration.DestHost, ...
        UDPPort=PacketFramework.Configuration.DestPort, in=Data, ...
        insize=NBytes);
    end

endfunction






function [sim,PacketFramework] = ld_PF_Finalise(sim,PacketFramework) // PARSEDOCU_BLOCK
    // 
    // Finalise the instance.
    // 
    // 


    function [sim, outlist, userdata] = SelectCaseSendNewConfigAvailable(sim, inlist, Ncase, casename, userdata)
        // This function is called multiple times -- once to define each case
        // At runtime, all cases will become different nested simulations of 
        // which only one is active a a time. 

        printf("Defining case %s (#%d) ...\n", casename, Ncase );

        // define names for the first event in the simulation
        events = 0;

        //  pause;

        PacketFramework = userdata(1);

        // print out some state information
        //  [sim] = ld_printf(sim, events, in=in1, str="case"+string(casename)+": in1", insize=1);

        // sample data for the output
        [sim, dummy] = ld_const(sim, 0, 999);

        // The signals "active_state" is used to indicate state switching: A value > 0 means the 
        // the state enumed by "active_state" shall be activated in the next time step.
        // A value less or equal to zero causes the statemachine to stay in its currently active
        // state

        select Ncase
        case 1 // Finished
            //[sim] = ld_printf(sim, events, dummy, "case "+string(casename)+" : Config already sent ", 1);

        case 2 // Send
            //[sim] = ld_printf(sim, events, dummy, "case "+string(casename)+" : Config is sent to PaPi ", 1);
            [sim] = ld_PF_SendNewConfigAvailableUDP(sim, PacketFramework);

        end

        // the user defined output signals of this nested simulation
        outlist = list(dummy);
        userdata(1) = PacketFramework;
    endfunction

    // The main real-time thread
    function [sim,PacketFramework] = ld_PF_InitUDP(sim, PacketFramework)

        function [sim, outlist, userdata] = UDPReceiverThread(sim, inlist, userdata)
            // This will run in a thread. Each time a UDP-packet is received 
            // one simulation step is performed. Herein, the packet is parsed
            // and the contained parameters are stored into a memory.


            // select case function for the different received PaPi Commands (SenderID)
            function [sim, outlist, userdata] = SelectCasePaPiCmd(sim, inlist, Ncase, casename, userdata)
                // This function is called multiple times -- once to define each case
                // At runtime, all cases will become different nested simulations of 
                // which only one is active a a time. 

                printf("Defining case %s (#%d) ...\n", casename, Ncase );

                // define names for the first event in the simulation
                events = 0;

                DisAsm_ = list();
                DisAsm_(4) = inlist(4);
                [sim, DisAsm_(1)] = ld_Int32ToFloat(sim, 0, inlist(1) );
                [sim, DisAsm_(2)] = ld_Int32ToFloat(sim, 0, inlist(2) );
                [sim, DisAsm_(3)] = ld_Int32ToFloat(sim, 0, inlist(3) );
                PacketFramework = userdata(1);
                ParameterMemory = PacketFramework.ParameterMemory;
                TotalElemetsPerPacket = PacketFramework.TotalElemetsPerPacket; // number of doubles values that fit into one UDP-packet with maximal size of 1400 bytes
                InstanceName = PacketFramework.InstanceName;
                // print out some state information
                //[sim] = ld_printf(sim, events, in=DisAsm_(3), str="case "+string(casename)+" with Ncase = "+string(Ncase)+" : SourceID", insize=1);

                // sample data for the output
                [sim, dummy] = ld_const(sim, 0, 9999);

                // The signals "active_state" is used to indicate state switching: A value > 0 means the 
                // the state enumed by "active_state" shall be activated in the next time step.
                // A value less or equal to zero causes the statemachine to stay in its currently active
                // state
                select Ncase
                case 1
                    [sim, memofs] = ld_ArrayInt32(sim, 0, array=ParameterMemory.MemoryOfs, in=inlist(3) );
                    [sim, Nelements] = ld_ArrayInt32(sim, 0, array=ParameterMemory.Sizes, in=inlist(3) );

                    [sim, memofs_] = ld_Int32ToFloat(sim, 0, memofs );
                    [sim, Nelements_] = ld_Int32ToFloat(sim, 0, Nelements );

                    if PacketFramework.Configuration.debugmode then 
                        // print the contents of the packet
                        [sim] = ld_printf(sim, 0, DisAsm_(1), "DisAsm(1) (SenderID)       = ", 1);
                        [sim] = ld_printf(sim, 0, DisAsm_(2), "DisAsm(2) (Packet Counter) = ", 1);
                        [sim] = ld_printf(sim, 0, DisAsm_(3), "DisAsm(3) (SourceID)       = ", 1);
                        [sim] = ld_printf(sim, 0, DisAsm_(4), "DisAsm(4) (Signal)         = ", TotalElemetsPerPacket);

                        [sim] = ld_printf(sim, 0, memofs_ ,  "memofs                    = ", 1);
                        [sim] = ld_printf(sim, 0, memofs_ ,  "Nelements                 = ", 1);
                    end

                    // Store the input data into a shared memory
                    [sim] = ld_WriteMemory2(sim, 0, data=inlist(4), index=memofs, ElementsToWrite=Nelements, ...
                    ident_str=InstanceName+"Memory", datatype=ORTD.DATATYPE_FLOAT, MaxElements=TotalElemetsPerPacket );

                case 2
                    [sim] = ld_printf(sim, 0, DisAsm_(3), "Give Config (SourceID)       = ", 1);
                    [sim] = ld_PF_SendConfigUDP(sim, PacketFramework);

                case 3
                    [sim] = ld_printf(sim, events, in=DisAsm_(3), str="case "+string(casename)+" : Wrong SourceID", insize=1);
                end

                // the user defined output signals of this nested simulation
                outlist = list(dummy);
                userdata(1) = PacketFramework;
            endfunction


            PacketFramework = userdata(1);

            TotalElemetsPerPacket = PacketFramework.TotalElemetsPerPacket; // number of doubles values that fit into one UDP-packet with maximal size of 1400 bytes
            InstanceName = PacketFramework.InstanceName;
            PacketSize = PacketFramework.PacketSize;
            // Sync the simulation to incomming UDP-packets
            [sim, Data, SrcAddr] = ld_UDPSocket_Recv(sim, 0, ObjectIdentifyer=InstanceName+"aSocket", outsize=PacketSize );

            // disassemble packet's structure
            [sim, DisAsm] = ld_DisassembleData(sim, 0, in=Data, ...
            outsizes=[1,1,1,TotalElemetsPerPacket], ...
            outtypes=[ ORTD.DATATYPE_INT32, ORTD.DATATYPE_INT32, ORTD.DATATYPE_INT32, ORTD.DATATYPE_FLOAT ] );

            //
            // Decice on what should be done depending on the field sourceID
            //


            // check if the request is to change a parameter
            [sim, sourceID] = ld_Int32ToFloat(sim, 0, DisAsm(3) );
            [sim, sourceIDPlus1] = ld_add_ofs(sim, ev, sourceID, 1);
            [sim, selectSignal_checkGtZero] = ld_cond_overwrite(sim, ev, sourceID, sourceIDPlus1, 1);

            // check if the request is to send the config to Papi
            [sim, selectSignal_notCheckGtZero] = ld_not(sim, ev, sourceIDPlus1);
            [sim, selectSignal_checkMinThreeInt32] = ld_CompareEqInt32(sim, ev, DisAsm(3), -3);
            [sim, selectSignal_checkMinThree] = ld_Int32ToFloat(sim, ev, selectSignal_checkMinThreeInt32);
            [sim, selectSignal_checked] = ld_cond_overwrite(sim, ev, selectSignal_checkGtZero, selectSignal_checkMinThree, 2);

            // check if (unused by now)
            [sim, selectSignal_notCheckMinThree] = ld_not(sim, ev, selectSignal_checkMinThree);
            [sim, selectSignal_undefined] = ld_and(sim, ev, list(selectSignal_notCheckGtZero, selectSignal_notCheckMinThree));
            [sim, selectSignal_checkedSecure] = ld_cond_overwrite(sim, ev, selectSignal_checked, selectSignal_undefined, 3);


            [sim, selectSignal_checkedSecureInt32] = ld_roundInt32(sim, ev, selectSignal_checkedSecure);

            // set-up the states for each PaPi Command represented by nested simulations
            [sim, outlist, userdataSelectCasePaPiCmd] = ld_CaseSwitchNest(sim, ev, ...
            inlist=DisAsm, ..
            insizes=[1,1,1,TotalElemetsPerPacket], outsizes=[1], ... 
            intypes=[ ORTD.DATATYPE_INT32, ORTD.DATATYPE_INT32, ORTD.DATATYPE_INT32, ORTD.DATATYPE_FLOAT ], outtypes=[ORTD.DATATYPE_FLOAT], ...
            CaseSwitch_fn=SelectCasePaPiCmd, SimnestName="SwitchSelectPaPiCmd", DirectFeedthrough=%t, SelectSignal=selectSignal_checkedSecureInt32, list("Param", "GiveConfig", "Undefined"), list(PacketFramework)  );

            PacketFramework = userdataSelectCasePaPiCmd(1);

            // output of schematic
            outlist = list();
            userdata(1) = PacketFramework;
        endfunction



        // start the node.js service from the subfolder webinterface
        //[sim, out] = ld_startproc2(sim, 0, exepath="./webappUDP.sh", chpwd="webinterface", prio=0, whentorun=0);

        TotalMemorySize = sum(PacketFramework.ParameterMemory.Sizes);
        InstanceName = PacketFramework.InstanceName;
        InitValues =  PacketFramework.ParameterMemory.InitValues;
        disp(InitValues);
        //	// Open an UDP-Port in server mode
        //	[sim] = ld_UDPSocket_shObj(sim, 0, ObjectIdentifyer=InstanceName+"aSocket", Visibility=0, ...
        //                                  hostname=PacketFramework.Configuration.LocalSocketHost, ...
        //                                  UDPPort=PacketFramework.Configuration.LocalSocketPort);

        // initialise a global memory for storing the input data for the computation
        [sim] = ld_global_memory(sim, 0, ident_str=InstanceName+"Memory", ... 
        datatype=ORTD.DATATYPE_FLOAT, len=TotalMemorySize, ...
        initial_data=[InitValues], ... 
        visibility='global', useMutex=1);

        // Create thread for the receiver
        ThreadPrioStruct.prio1=ORTD.ORTD_RT_NORMALTASK, ThreadPrioStruct.prio2=0, ThreadPrioStruct.cpu = -1;
        [sim, startcalc] = ld_const(sim, 0, 1); // triggers your computation during each time step
        [sim, outlist, computation_finished, userdata] = ld_async_simulation(sim, 0, ...
        inlist=list(), ...
        insizes=[], outsizes=[], ...
        intypes=[], outtypes=[], ...
        nested_fn = UDPReceiverThread, ...
        TriggerSignal=startcalc, name=InstanceName+"Thread1", ...
        ThreadPrioStruct, userdata=list(PacketFramework) );

        PacketFramework = userdata(1);

    endfunction


    // calc memory
    MemoryOfs = [];
    Sizes = [];
    InitValues = [];
    // go through all parameters and create memories for all
    for i=1:length(PacketFramework.Parameters)
        P = PacketFramework.Parameters(i);

        Sizes = [Sizes; P.NValues];
        MemoryOfs = [MemoryOfs; P.MemoryOfs];
        InitValues = [InitValues; P.InitialValue(:)];
    end

    PacketFramework.ParameterMemory.MemoryOfs = MemoryOfs;
    PacketFramework.ParameterMemory.Sizes = Sizes;
    PacketFramework.ParameterMemory.InitValues = InitValues;

    // udp
    [sim] = ld_PF_InitUDP(sim, PacketFramework);

    // Send to group update notifications for each group (currently only one possible)
    [sim] = ld_PF_SendGroupFinshUDP(sim, PacketFramework, GroupID=0);


    [sim, initSelect] = ld_initimpuls(sim, ev);
    [sim, selectNewConfig] = ld_add_ofs(sim, ev, initSelect, 1);
    //[sim] = ld_printf(sim, ev, selectNewConfig,  ORTD.termcode.red + "Select New Config Signal" +  ORTD.termcode.reset, 1);
    [sim, selectNewConfigInt32] = ld_roundInt32(sim, ev, selectNewConfig);
    // set-up the states to send a Signal to PaPi that a new config is available only in the first time step represented by nested simulations
    [sim, outlist, userdata] = ld_CaseSwitchNest(sim, ev, ...
    inlist=list(), ..
    insizes=[], outsizes=[1], ... 
    intypes=[], outtypes=[ORTD.DATATYPE_FLOAT], ...
    CaseSwitch_fn=SelectCaseSendNewConfigAvailable, SimnestName="SelectCaseSendNewConfigAvailable", DirectFeedthrough=%t, SelectSignal=selectNewConfigInt32, list("Finished", "Send"), list(PacketFramework)  );

    PacketFramework = userdata(1);

endfunction


function str=ld_PF_Export_str(PacketFramework)
    //     Added possibility to add GUI-configurations on 5.3.15

    
    
    
    
    
    
    
    // check if there is a GUI to be set-up in Papi
    if  isfield(PacketFramework, 'PaPIConfig') then
        PaPIConfigstr = struct2json(PacketFramework.PaPIConfig)
    else
        PaPIConfigstr = '{}';
    end

    
    LineBreakChar = ''; // char(10);


    str=' {""SourcesConfig"" : {'+LineBreakChar;

    for i=1:length(PacketFramework.Sources)


        SourceID = PacketFramework.Sources(i).SourceID;
        SourceName =  PacketFramework.Sources(i).SourceName;
//        disp(SourceID );
//        disp( SourceName );

        // Add optional Demultiplexing information
        if isfield( PacketFramework.Sources(i), 'Demux' );
            DmxStr = struct2json(PacketFramework.Sources(i).Demux);
        else
            DmxStr = "{}";
        end
        
        line=sprintf(" ""%s"" : { ""SourceName"" : ""%s"" , ""NValues_send"" : ""%s"", ""datatype"" : ""%s"", ""Demux"" : %s  } ", ...
        string(PacketFramework.Sources(i).SourceID), ...
        string(PacketFramework.Sources(i).SourceName), ...
        string(PacketFramework.Sources(i).NValues_send), ...
        string(PacketFramework.Sources(i).datatype), DmxStr );


        if i==length(PacketFramework.Sources)
            // finalise the last entry without ","
            printf('%s' , line);
            str=str+line + LineBreakChar;
        else
            printf('%s,' , line);
            str=str+line+',' + LineBreakChar;
        end


    end



    str=str+'} , ' + LineBreakChar + ' ""ParametersConfig"" : {' + LineBreakChar;

    // go through all parameters and create memories for all
    for i=1:length(PacketFramework.Parameters)

        printf("export of parameter %s \n",PacketFramework.Parameters(i).ParameterName );

        initVal = PacketFramework.Parameters(i).InitialValue;
        if length(initVal) == 1 then
            initvalstr = string(initVal);
        else
            initvalstr = sci2exp(initVal(:)');
        end

        line=sprintf(" ""%s"" : { ""ParameterName"" : ""%s"" , ""NValues"" : ""%s"", ""datatype"" : ""%s"", ""initial_value"" : %s  } ", ...
        string(PacketFramework.Parameters(i).ParameterID), ...
        string(PacketFramework.Parameters(i).ParameterName), ...
        string(PacketFramework.Parameters(i).NValues), ...
        string(PacketFramework.Parameters(i).datatype), ...
        initvalstr );


        if i==length(PacketFramework.Parameters) 
            // finalise the last entry without ","
            printf('%s' , line);
            str=str+line + LineBreakChar;
        else
            printf('%s,' , line);
            str=str+line+',' + LineBreakChar;
        end


    end  
    str=str+'}, '+LineBreakChar+ """" + 'PaPIConfig' + """" + ' : ' + PaPIConfigstr + LineBreakChar +  '}'; // 

    // print the configuration to be send to Papi
    disp(str);

endfunction

function ld_PF_Export_js(PacketFramework, fname) // PARSEDOCU_BLOCK
    // 
    // Export configuration of the defined protocoll (Sources, Parameters) 
    // into JSON-format. This is to be used by software that shall communicate 
    // to the real-time system.
    // 
    // fname - The file name
    // 
    // 
    str=ld_PF_Export_str(PacketFramework);

    fd = mopen(fname,'wt');
    mfprintf(fd,'%s', str);
    mclose(fd);

endfunction

// 
// Added 27.3.14
// 

function [sim, PacketFramework, SourceID]=ld_SendPacketReserve(sim, PacketFramework, NValues_send, datatype, SourceName)
    [PacketFramework,SourceID] = ld_PF_addsource(PacketFramework, NValues_send, datatype, SourceName, 0 );
endfunction

function [sim, PacketFramework]=ld_SendPacket2(sim, PacketFramework, Signal, SourceName)
    // find Sourcename
    index  = -1;
    for i=1:length(PacketFramework.Sources)
        S = PacketFramework.Sources(i);
        if S.SourceName == SourceName
            index = i;
            printf(" %s found at index %d Nvalues %d\n", SourceName, index, S.NValues_send);
            break;
        end
    end

    if index == -1
        printf("SourceName = %s\n", SourceName);
        error("SourceName not found! This source must be reservated using ld_SendPacketReserve");
    end

    [sim]=ld_PF_ISendUDP(sim, PacketFramework, Signal, S.NValues_send, S.datatype, S.SourceID);
endfunction



function [sim, PacketFramework, ParameterID]=ld_PF_ParameterReserve(sim, PacketFramework, NValues, datatype, ParameterName)
    [PacketFramework,ParameterID,MemoryOfs] = ld_PF_addparameter(PacketFramework, NValues, datatype, ParameterName);   
endfunction


function [sim, PacketFramework, Parameter]=ld_PF_Parameter2(sim, PacketFramework, ParameterName)
    // find Sourcename
    index  = -1;
    for i=1:length(PacketFramework.Parameters)
        P = PacketFramework.Parameters(i);
        if P.ParameterName == ParameterName
            index = i;
            printf(" %s found at index %d Nvalues %d\n", ParameterName, index, P.NValues);
            break;
        end
    end

    if index == -1
        printf("ParameterName = %s\n", ParameterName);
        error("ParameterName not found! This source must be reservated using ld_PF_ParameterReserve");
    end

    // read data from global memory
    [sim, readI] = ld_const(sim, 0, P.MemoryOfs); // start at index 1
    [sim, Parameter] = ld_read_global_memory(sim, 0, index=readI, ident_str=PacketFramework.InstanceName+"Memory", ...
    P.datatype, P.NValues);
endfunction



