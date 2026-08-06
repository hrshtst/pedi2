#!/bin/sh

# find all executable files (excluding script files etc.)
# make sure only tests in the working directory
FOUND_BINS=`find . -maxdepth 1 -type f -executable -exec file -i '{}' \; | grep -E 'x-(pie-)?executable; charset=binary' | sed -e "s/:.*$//g"`

# run all tests
SUM_RET=0
for i in $FOUND_BINS
do
    NAME=`echo $i | sed -e "s/^..//g"`
    echo "Running $NAME...."
    $i
    RET=$?
    SUM_RET=`expr $SUM_RET + $RET`
    if [ $RET -gt 0 ]; then
	set -- "$@" $NAME
    fi
done

if [ $SUM_RET -gt 0 ]; then
    printf "\n"
    printf "\033[31m==============================\n"
    printf "\033[31m There are some failed tests!\n"
    printf "\033[31m   * %s\n" "$@"
    printf "\033[31m==============================\n"
fi

# propagate failure to the caller (e.g. make test in CI)
[ $SUM_RET -gt 0 ] && exit 1
exit 0
