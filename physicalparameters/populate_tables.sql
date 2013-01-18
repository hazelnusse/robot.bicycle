-- This file includes all the measurements that were written down on Jason's
-- Google doc handout.

-- Populate the Fork Center of Mass table
insert into parametermeasurements.forkcenterofmass values(1, radians(118.4), -0.2477, 'cantilever brake bridge fixture, forgot headset locknut');
insert into parametermeasurements.forkcenterofmass values(2, radians(149.8), -0.1234, 'cantilever brake bridge fixture');
insert into parametermeasurements.forkcenterofmass values(3, radians( 23.3), -0.1645, 'centerpull brake fixture in front of steer tube');
insert into parametermeasurements.forkcenterofmass values(4, radians(-27.8),  0.1100, 'centerpull brake fixture behind steer tube');
insert into parametermeasurements.forkcenterofmass values(5, radians(172.0),  0.0000, 'hub fixture');

-- Populate the Frame Center of Mass table
insert into parametermeasurements.framecenterofmass values(1, radians( -95.8),  0.1915, 'rear cantilever brake bridge fixture');
insert into parametermeasurements.framecenterofmass values(2, radians(-168.3), -0.5280, 'lower portion of down tube');
insert into parametermeasurements.framecenterofmass values(3, radians(-131.2), -0.2015, 'seat tube');
insert into parametermeasurements.framecenterofmass values(4, radians( 126.7), -0.5520, 'upper portion of down tube');
insert into parametermeasurements.framecenterofmass values(5, radians(-113.2),  0.0000, 'hub fixture');

-- Populate Mass measurements
insert into parametermeasurements.massmeasurements values(1, 'Rear Wheel', 4.4);
insert into parametermeasurements.massmeasurements values(2, 'Front Wheel', 1.5);
insert into parametermeasurements.massmeasurements values(3, 'Fork', 1.45);
insert into parametermeasurements.massmeasurements values(4, 'Frame', 29.65);

-- Populate wheel compound pendulum measurements
insert into parametermeasurements.compoundpendulumLengths values(1, 'Rear Wheel', 0.280289);
insert into parametermeasurements.compoundpendulumLengths values(2, 'Front Wheel', 0.2898775);

-- Populate the perpendicular distances (from machine shop measurements of
  -- frame and fork)
insert into parametermeasurements.axleoffsetmeasurements values(1,37.943*0.0254, 1.8825*0.0254, 13.521*0.0254);


-- Populate wheel roll out tables
insert into parametermeasurements.wheelrolloutmeasurements values (1, 'Rear Wheel', 10, (69.0*12.0 + 3.0835)*0.0254, 120.0);
insert into parametermeasurements.wheelrolloutmeasurements values (2, 'Rear Wheel', 10, (69.0*12.0 + 2.9480)*0.0254, 120.0);
insert into parametermeasurements.wheelrolloutmeasurements values (3, 'Rear Wheel', 10, (69.0*12.0 + 2.7660)*0.0254, 120.0);
insert into parametermeasurements.wheelrolloutmeasurements values (4, 'Front Wheel', 10, (69.0*12.0 + 2.9450)*0.0254, 120.0);
insert into parametermeasurements.wheelrolloutmeasurements values (5, 'Front Wheel', 10, (69.0*12.0 + 2.7130)*0.0254, 120.0);
insert into parametermeasurements.wheelrolloutmeasurements values (6, 'Front Wheel', 10, (69.0*12.0 + 2.8115)*0.0254, 120.0);

-- Populate torsional spring calibration rod table
insert into parametermeasurements.calibrationrods values(1, 4.65, (0.03009 + 0.03010 + 0.03012)/3.0, 0.0, 0.8355);
