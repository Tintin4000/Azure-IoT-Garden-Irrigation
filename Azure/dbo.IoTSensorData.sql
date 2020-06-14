CREATE TABLE [dbo].[IoTSensorData] (
    [Id]               BIGINT        IDENTITY (1, 1) NOT NULL,
    [sensordatetime]   DATETIME2 (7) NOT NULL,
    [sensorid]         NCHAR (25)    NOT NULL,
    [devicetype]       NCHAR (25)    DEFAULT ('N/A') NOT NULL,
    [location]         NCHAR (25)    DEFAULT ('N/A') NOT NULL,
    [battery_volt]     FLOAT (53)    DEFAULT ((0)) NULL,
    [status] TINYINT NULL, 
    [air_temperature]  FLOAT (53)    NULL,
    [air_humidity]     FLOAT (53)    NULL,
    [soil_temperature] FLOAT (53)    NULL,
    [soil_humidity]    FLOAT (53)    NULL,    
    CONSTRAINT [PK_ID] PRIMARY KEY CLUSTERED ([Id] ASC)
);
