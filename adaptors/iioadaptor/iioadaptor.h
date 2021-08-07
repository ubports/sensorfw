/**
   @file iioadaptor.h
   @brief IioAdaptor based on SysfsAdaptor

   <p>
   Copyright (C) 2009-2010 Nokia Corporation
   Copyright (C) 2012 Tuomas Kulve
   Copyright (C) 2016 Canonical

   @author Tuomas Kulve <tuomas@kulve.fi>
   @author Lorn Potter <lorn.potter@canonical.com>

   Sensord is free software; you can redistribute it and/or modify
   it under the terms of the GNU Lesser General Public License
   version 2.1 as published by the Free Software Foundation.

   Sensord is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with Sensord.  If not, see <http://www.gnu.org/licenses/>.
   </p>
*/

#ifndef IIOADAPTOR_H
#define IIOADAPTOR_H

#include <sysfsadaptor.h>
#include <datatypes/orientationdata.h>

// FIXME: shouldn't assume any number of channels per device
#define IIO_MAX_DEVICE_CHANNELS     20

// FIXME: no idea what would be reasonable length
#define IIO_BUFFER_LEN              256

/**
 * @brief Adaptor for Industrial I/O.
 *
 * Adaptor for Industrial I/O. Uses SysFs driver interface in
 * polling mode, i.e. values are read with given constant interval.
 *
 * Driver interface is located in @e /sys/bus/iio/devices/iio:deviceX/ .
 * <ul><li>@e angular_rate filehandle provides measurement values.</li></ul>
 * No other filehandles are currently in use by this adaptor.
 */
class IioAdaptor : public SysfsAdaptor
{
    Q_OBJECT
    enum IioSensorType {
        IIO_ACCELEROMETER = 1, // accel_3d
        IIO_GYROSCOPE, // gyro_3d
        IIO_MAGNETOMETER, // magn_3d
        IIO_ROTATION, // dev_rotation, quaternion
        IIO_ALS, // als
        IIO_TILT, // incli_3d
        IIO_PROXIMITY // proximity als
    };

    struct iio_device {
      QString name;
      int channels;
      int channel_bytes[IIO_MAX_DEVICE_CHANNELS];
      qreal scale;
      qreal offset;
      int frequency;
      QList<float> frequencyList;
      QString devicePath;
      int index;
      IioSensorType sensorType;
      QString channelTypeName;
    };

public:
    /**
     * Factory method for gaining a new instance of this adaptor class.
     *
     * @param id Identifier for the adaptor.
     * @return A pointer to created adaptor with base class type.
     */
    static DeviceAdaptor *factoryMethod(const QString& id)
    {
        return new IioAdaptor(id);
    }

    virtual bool startSensor();
    virtual void stopSensor();
//    virtual bool standby();
//    virtual bool resume();

protected:

    /**
     * Constructor. Protected to force externals to use factory method.
     *
     * @param id Identifier for the adaptor.
     */
    IioAdaptor(const QString &id);

    /**
     * Destructor.
     */
    ~IioAdaptor();


    bool setInterval(const unsigned int value, const int sessionId);
    //  unsigned int interval() const;

private:

    /**
     * Read and process data. Run when sysfsadaptor has detected new
     * available data.
     *
     * @param pathId PathId for the file that had event.
     * @param fd Open file descriptor with new data. See
     *           #SysfsAdaptor::processSample()
     */
    void processSample(int pathId, int fd);

    int findSensor(const QString &name);
    bool deviceEnable(int device, int enable);

    bool sysfsWriteInt(QString filename, int val);
    QString sysfsReadString(QString filename);
    int sysfsReadInt(QString filename);
    int scanElementsEnable(int device, int enable);
    int deviceChannelParseBytes(QString filename);

    // Device number for the sensor (-1 if not found)
    int devNodeNumber;

    int proximityThreshold;

    DeviceAdaptorRingBuffer<TimedXyzData>* accelerometerBuffer_;
    DeviceAdaptorRingBuffer<TimedXyzData>* gyroscopeBuffer_;
    DeviceAdaptorRingBuffer<TimedUnsigned>* alsBuffer_;
    DeviceAdaptorRingBuffer<CalibratedMagneticFieldData>* magnetometerBuffer_;
    DeviceAdaptorRingBuffer<ProximityData>* proximityBuffer_;

    iio_device iioDevice;

    QString deviceId;

    TimedXyzData* accelData;
    TimedXyzData* gyroData;
    CalibratedMagneticFieldData *calMagData;
    TimedUnsigned *uData;
    ProximityData *proximityData;

private slots:
    void setup();
};

#endif
