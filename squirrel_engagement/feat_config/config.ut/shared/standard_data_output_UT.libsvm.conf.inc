///////////////////////////////////////////////////////////////////////////////////////
///////// > openSMILE configuration file for data output <           //////////////////
/////////                                                            //////////////////
///////// (c) audEERING GmbH,                                        //////////////////
/////////     All rights reserverd.                                  //////////////////
///////////////////////////////////////////////////////////////////////////////////////


/*
   This file can be included as data output file for standard feature
   extraction configuration files. It provides commandline options
   for the batch extraction GUI, and supports LLD and Functionals (summaries)
   saving.

   It requires the main extrator configuration file to provide the following
   data memory levels:  lld, lld_de, and func
 */

  //////////////////////////////////////////////////////////////////////
 ///////////////////  data output configuration  //////////////////////
//////////////////////////////////////////////////////////////////////

;;;;;;;;; output LLD features to CSV ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

[componentInstances:cComponentManager]
instance[svmsink].type=cLibsvmSink

[svmsink:cLibsvmSink]
reader.dmLevel=func
timestamp=0
append=1
filename=\cm[output(O){?}:name of libsvm output file, set to a valid filename to enable this output sink]