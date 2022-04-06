#!/bin/bash

LIMELIGHT_STREAM="http://10.21.68.80:5800/stream.mjpg"
FILENAME=$1  # first cli argument
file_iter=0

get_file_name() {
    echo "getting filename"
    filename="${FILENAME}-${file_iter}.mkv"
    while [ -f $filename ]; do
        ((file_iter+=1))
        filename="${FILENAME}-${file_iter}.mkv"
    done
}

# group_segment() {
#     VIDEO="full_${FILENAME}.mkv"
#     for segment in $FILENAME*.mkv; do
#         echo $segment
#         cat $segment >> $VIDEO
#         rm $segment
#     done
# }
# trap group_segment 0 1 2 127

exitcode=0
# while not keyboard interrupt
while [ $exitcode -ne 127 ]; do
        get_file_name

        if [ $exitcode -ne 0 ]; then
                printf "Got exit code ${exitcode}!  Retrying recording...\n"
        fi

        printf "Recording to file ${filename}\n\n\n"
        ffmpeg -loglevel 32 -i "$LIMELIGHT_STREAM" "$filename"
        exitcode=$?
done

printf "Exiting... grouping video segments into final file.\n"