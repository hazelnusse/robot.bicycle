#!/bin/bash

if [ -e /run/media/hazelnusse/MICROSD_0/samples.dat ]; then
  datafile="/run/media/hazelnusse/MICROSD_0/samples.dat"
  base_fn="samples_$(date +%H%M)_$(date +%d%^b%Y)"
  raw_data_file="./collected_data/${base_fn}.dat"
  converted_data_file="./collected_data_converted/${base_fn}_converted.dat"
  cp ${datafile} ${raw_data_file}
  if [ $? -gt 0 ]; then
    echo "Failed to copy raw data."
    exit 1
  fi
  ./dataprocessing-build/convertsamples ${raw_data_file} ${converted_data_file}
  if [ $? -gt 0 ]; then
    echo "Failed to convert data."
    exit 1
  fi
  echo "Raw data saved to: ${raw_data_file}"
  echo "Converted data saved to: ${converted_data_file}"
else
  echo "No SD card available or no samples.dat file present."
fi

