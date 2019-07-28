CREATE TABLE [dbo].[IoTSensorData] (
    [Id]               BIGINT        IDENTITY (1, 1) NOT NULL,
    [sensordatetime]   DATETIME2 (7) NOT NULL,
    [deviceid]         NCHAR (25)    NOT NULL,
    [devicetype]       NCHAR (25)    NOT NULL DEFAULT 'N/A',
    [location]         NCHAR (25)    NOT NULL DEFAULT 'N/A',
    [air_temperature]  FLOAT (53)    NULL,
    [air_humidity]     FLOAT (53)    NULL,
    [soil_temperature] FLOAT (53)    NULL,
    [soil_humidity]    FLOAT (53)    NULL,
    CONSTRAINT [PK_ID] PRIMARY KEY CLUSTERED ([Id] ASC)
);

