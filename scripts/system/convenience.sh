echo "----------------------------------------------------------------------------------- \n"


echo "\nAdding Convenience Scripts to Your linux Commands \n"

cp beta      /usr/local/bin
cp build     /usr/local/bin
cp launch    /usr/local/bin
cp s         /usr/local/bin

ls /usr/local/bin

echo "\n\nScripts added! You now have access to commands beta, build, launch, and s \n\n"

echo "----------------------------------------------------------------------------------- \n\n"

echo "beta (directory) - clones the github into that directory \n"
echo "build (node) - runs colcon build --packages-select so you don't have to type it \n"
echo "launch - allows you to source and launch automatically (saves about 6 seconds every time you launch) \n"
echo "s - source so you don't have to remember the command \n"
