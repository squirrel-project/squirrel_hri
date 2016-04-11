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
instance[arffsink].type=cArffSink

[arffsink:cArffSink]
reader.dmLevel=func
 ; do not print "frameIndex" attribute to ARFF file
frameIndex = 0
frameTime = 0
 ; name of output file as commandline option
filename=\cm[arffout(O){output.arff}:name of WEKA Arff output file]
 ; name of @relation in the ARFF file
relation=\cm[corpus{SMILEfeatures}:corpus name, arff relation]

 
 ; name of the current instance (usually file name of input wave file)
;instanceName=\cm[instname(N){noname}:name of arff instance]
 ;; use this line instead of the above to always set the instance name to the
 ;; name of the input wave file
;;instanceName=\cm[inputfile]
class[0].name = class
class[0].type = string
target[0].all = \cm[classlabel{neutral}:instance nominal class label]

class[1].name = uid
class[1].type = string
target[1].all = \cm[words(w){empty}:transcribed text]

append=1


//////---------------------- END -------------------------///////
