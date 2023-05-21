# If you have questions about this script ask Talor
# Directory containing the motor_tests
current_dir=current_directory=$(pwd)
dir_name="/motor_tests/"
test_dir="$current_dir$dir_name"
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

run_commands

esac
