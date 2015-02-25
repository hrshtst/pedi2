# find all executable files (excluding script files etc.)
found_bins=`find . -maxdepth 1 -type f -executable -exec file -i '{}' \; | grep 'x-executable; charset=binary' | sed -e "s/:.*$//g"`

# run all binaries
for i in $found_bins
do
    name=`echo $i | sed -e "s/^..//g"`
    echo "Running $name...."
    $i
done
