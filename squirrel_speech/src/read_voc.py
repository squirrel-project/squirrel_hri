#!/usr/bin/python

import sys 
from std_msgs.msg import String

def read_voc(voc_path): 

    print "Reading Vocabulary from: %r" % voc_path
    rows = open(voc_path).read().split("\n")

    synonyms = {}
    key_num=0;
    syn_num=0;

    for r in rows:
        head_and_tail = r.split(":")
        head = head_and_tail[0]
        #print("Head: " + head)
        key_num = key_num+1;

        
        if(len(head_and_tail)>1 ):
            tail = head_and_tail[1]
            #print("\tTail: " + tail)
            synonyms[head]=tail.split(",")
            syn_num = syn_num + 1;

    print("found " + str(key_num) + " triggers - of them " + str(syn_num) + " have synonyms")  


    return synonyms
        





