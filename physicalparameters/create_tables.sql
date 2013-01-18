create schema parametermeasurements;
create schema parameters;
create table parametermeasurements.forkcenterofmass
(
  id              serial  primary key,
  alpha           float8  not null,
  a               float8  not null,
  notes           text
);

create table parametermeasurements.framecenterofmass
(
  id              serial  primary key,
  alpha           float8  not null,
  a               float8  not null,
  notes           text
);

create table parametermeasurements.forktorsionalpendulumtimeseries
(
  id              serial  primary key,
  configuration   integer references parametermeasurements.forkcenterofmass(id),
  samplefrequency integer not null,
  duration        integer not null,
  voltage         float8[],
  notes           text
);

create table parametermeasurements.forkcompoundpendulumtimeseries
(
  id              serial  primary key,
  configuration   integer references parametermeasurements.forkcenterofmass(id),
  samplefrequency integer not null,
  duration        integer not null,
  voltage         float8[],
  notes           text
);

create table parametermeasurements.frametorsionalpendulumtimeseries
(
  id              serial  primary key,
  configuration   integer references parametermeasurements.framecenterofmass(id),
  samplefrequency integer not null,
  duration        integer not null,
  voltage         float8[],
  notes           text
);

create table parametermeasurements.framecompoundpendulumtimeseries
(
  id              serial  primary key,
  configuration   integer references parametermeasurements.framecenterofmass(id),
  samplefrequency integer not null,
  duration        integer not null,
  voltage         float8[],
  notes           text
);

create table parametermeasurements.rearwheelcompoundpendulumtimeseries
(
  id              serial  primary key,
  samplefrequency integer not null,
  duration        integer not null,
  voltage         float8[],
  notes           text
);

create table parametermeasurements.frontwheelcompoundpendulumtimeseries
(
  id              serial  primary key,
  samplefrequency integer not null,
  duration        integer not null,
  voltage         float8[],
  notes           text
);

create table parametermeasurements.rearwheeltorsionalpendulumtimeseries
(
  id              serial  primary key,
  samplefrequency integer not null,
  duration        integer not null,
  voltage         float8[],
  notes           text
);

create table parametermeasurements.frontwheeltorsionalpendulumtimeseries
(
  id              serial  primary key,
  samplefrequency integer not null,
  duration        integer not null,
  voltage         float8[],
  notes           text
);

create table parametermeasurements.massmeasurements
(
  id       serial primary key,
  body     text,
  mass     float8
);

create table parametermeasurements.axleoffsetmeasurements
(
  id  serial primary key,
  rearaxleoffset float8,
  frontaxleoffset float8,
  steeraxisoffset float8
);


create table parametermeasurements.wheelrolloutmeasurements
(
  id  serial primary key,
  body    text,
  revolutions integer,
  distance float8,
  tirepressure float8
);

create table parametermeasurements.compoundpendulumlengths
(
  id serial primary key,
  body text,
  length float8
);

create table parameters.wheels
(
  id          serial primary key,
  name        text,
  mass        float8,
  majorradius float8,
  minorradius float8,
  inertia_scalars   float8[6],
  notes       text
);

create table parameters.frames
(
  id                serial primary key,
  name              text,
  mass              float8,
  inertia_scalars   float8[6],
  com_position      float8[3],
  axle_offset       float8,
  notes             text
);

create table parameters.forks
(
  id                serial primary key,
  name              text,
  mass              float8,
  inertia_scalars   float8[6],
  com_position      float8[3],
  axle_offset       float8,
  notes             text
);

create table parameters.bicyclegyrostat
(
  id                  serial primary key,
  name                text,
  totalmass           float8,
  inertia_scalars     float8[6],
  spin_inertia        float8,
  com_position        float8[3]
);

create table parameters.bicycle
(
  id                  serial primary key,
  name                text,
  reargyrostat        integer references parameters.BicycleGyrostat(id),
  frontgyrostat       integer references parameters.BicycleGyrostat(id),
  steeraxisoffset     float8,
  gravity             float8,
  notes               text
);

create table parametermeasurements.calibrationrods
(
  id               serial primary key,
  mass             float8,
  outsidediameter  float8,
  insidediameter   float8,
  length           float8,
  notes            text
);

create table parametermeasurements.rodtorsionalpendulumtimeseries
(
  id              serial  primary key,
  rod             integer references parametermeasurements.calibrationrods(id),
  samplefrequency integer not null,
  duration        integer not null,
  voltage         float8[],
  notes           text
);
