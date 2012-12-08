#!/bin/bash

sd_card_dir="/media/sdcard"
mount ${sd_card_dir}
if [ $? -gt 0 ]; then
  echo "Failed mount sd card."
  exit 1
fi

ls -al ${sd_card_dir}
datafile="${sd_card_dir}/samples.dat"
base_fn="samples_$(date +%H%M)_$(date +%d%^b%Y)"
raw_data_file="./collected_data/${base_fn}.dat"
converted_data_file="./collected_data_converted/${base_fn}_converted.dat"
cp ${datafile} ${raw_data_file}
if [ $? -gt 0 ]; then
  echo "Failed to copy raw data."
  exit 1
fi

rm -rf ${datafile}

umount ${sd_card_dir}

./dataprocessing-build/convertsamples ${raw_data_file} ${converted_data_file}

if [ $? -gt 0 ]; then
  echo "Failed to convert data."
  exit 1
fi

echo "Raw data saved to: ${raw_data_file}"
echo "Converted data saved to: ${converted_data_file}"

exit 0
