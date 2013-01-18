#!/bin/bash
echo "Creating database..."
createdb robot_bicycle_parameters
echo "Creating tables..."
psql -d robot_bicycle_parameters -c "\i create_tables.sql"
echo "Populating tables..."
psql -d robot_bicycle_parameters -c "\i populate_tables.sql"
cd RawData/PeriodMeasurements/Fork
python populateTable.py
cd ../Frame
python populateTable.py
cd ../FrontWheel
python populateTable.py
cd ../RearWheel
python populateTable.py
cd ../Rod
python populateTable.py
echo "Tables populated"
