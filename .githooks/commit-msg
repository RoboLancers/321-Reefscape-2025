#!/bin/bash

# gets the git commit message based on the sha
gitmessage=`cat $1`
echo "Checking commit message: "
echo $gitmessage

####################### TEST STRINGS comment out line 13 to use #########################################
#gitmessage="feat sdasdsadsaas (AEROGEAR-asdsada)"
#gitmessage="feat(some txt): some txt (AEROGEAR-****)"
#gitmessage="docs(some txt): some txt (AEROGEAR-1234)"
#gitmessage="fix(some txt): some txt (AEROGEAR-5678)"
#########################################################################################################

# Checks gitmessage for string feat, fix, docs and breaking, if the messagecheck var is empty if fails
messagecheck=`echo $gitmessage | grep -w "feat\|fix\|docs\|breaking"`
if [ -z "$messagecheck" ]
then 
      echo "Your commit message must begin with one of the following"
      echo "  feat(feature-name)"
      echo "  fix(fix-name)"
      echo "  docs(docs-change)"
      echo " "
fi

messagecheck=`echo $gitmessage | grep ": "`
if  [ -z "$messagecheck" ]
then 
      echo "Your commit message has a formatting error please take note of special characters '():' position and use in the example below"
      echo "   type(some txt)"
      echo "Where 'type' is fix, feat, docs or breaking"
      echo " "
fi

# All checks run at the same time by pipeing from one grep to another
messagecheck=`echo $gitmessage | grep -w "feat\|fix\|docs\|breaking" | grep ": "`



# check to see if the messagecheck var is empty
if [ -z "$messagecheck" ]
then  
      echo "The commit message '$gitmessage' failed "
      echo "Please review the following :"
      echo " "
      echo $gitmessage
      echo " "
      exit 1
else
      echo "$messagecheck"
      echo "'$gitmessage' commit message passed"
      exit 0
fi