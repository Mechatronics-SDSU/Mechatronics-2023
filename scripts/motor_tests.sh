# If you have questions about this script ask Talor
# Directory containing the motor_tests
tests_dir=/home/mechatronics/master/Mechatronics-2023/scripts/motor_tests
pattern='([0-9]+)\(([^)]*)\)'
run_commands() {
  echo
  echo "Select commands"
  echo
  echo "Example: 2(1,2)3()4() (runs 2 with commands 1 and 2 and then runs 3, 4)" 
  read -p "Enter commands: " commands
  # Iterate through the the inputed commands char with arguments by char with arguments and run the script assoicated with that int
  while [[ $commands =~ $pattern ]]; do
      # The assoicated int with the script
      variable_name="script${BASH_REMATCH[1]}"
      # Get the args associated with each command
      tuple="${BASH_REMATCH[2]}"
      # Check if the tuple is empty
      if [[ -z $tuple ]]; then
        # Run selected command "${tuple}" is the args
        bash "${!variable_name}" "${tuple}"
      else
        # Run selected command
        bash "${!variable_name}"
      fi
      # Remove the matched portion from the string to process the next match
      commands=${commands#*"${BASH_REMATCH[0]}"}
  done
  echo
}
# Not done yet
walk_through_tests() {

}
c=0
# Iterate through each script in test scripts
for scripts in "$tests_dir"/*
do
  # Convert complete path to file into just the file name with the assoicated int
  echo "$scripts: $c" | sed "s/.*\///"
  # Define a variable like $def="This script does..." in a file in motor_tests and the defition will appear with the script
  # Note has to be $def= not $def =
  sed -n 's/^$def=\(.*\)/\1/p' < $scripts
  # Define the script by an asscoiated int in increasing order
  eval "script$c=$scripts";
  c=$((c+1));
  echo
done
read -n1 -p "Do you want to walk through any tests? [y,n]" doit 
case $doit in  
  y|Y) run_commands ;; 
  n|N) walk_through_tests;; 
esac
