/**

//    http://www.apache.org/licenses/LICENSE-2.0

   @file iioadaptor.cpp
   @brief IioAdaptor based on SysfsAdaptor

   <p>
   Copyright (C) 2009-2010 Nokia Corporation
   Copyright (C) 2012 Tuomas Kulve
   Copyright (C) 2012 Srdjan Markovic
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
#include <errno.h>

#include <libudev.h>

#include <logging.h>
#include <config.h>
#include <datatypes/utils.h>
#include <unistd.h>
#include <time.h>

#include "iioadaptor.h"
#include <sysfsadaptor.h>
#include <deviceadaptorringbuffer.h>
#include <QTextStream>
#include <QDir>
#include <QTimer>
#include <QDirIterator>
#include <qmath.h>
#include <QRegularExpression>

#include <deviceadaptor.h>
#include "datatypes/orientationdata.h"

#define GRAVITY             9.80665
#define REV_GRAVITY         0.101936799
#define RADIANS_TO_DEGREES 57.2957795

// Proximity sensor
#define PROXIMITY_DEFAULT_THRESHOLD 250
#define PROXIMITY_NEAR_VALUE 0
#define PROXIMITY_FAR_VALUE 100

/* Conversion of acceleration data to SI units (m/s^2) */
#define CONVERT_A_X(x)  ((float(x) / 1000) * (GRAVITY * -1.0))
#define CONVERT_A_Y(x)  ((float(x) / 1000) * (GRAVITY * 1.0))
#define CONVERT_A_Z(x)  ((float(x) / 1000) * (GRAVITY * 1.0))

/* Documentation for the iio sensors is given in
   https://www.kernel.org/doc/Documentation/ABI/testing/sysfs-bus-iio (1)
   In this document you will find the units which are used in explanation of values in sysfs-bus-iio
*/

IioAdaptor::IioAdaptor(const QString &id) :
        SysfsAdaptor(id, SysfsAdaptor::IntervalMode, true),
        deviceId(id)
{
    sensordLogD() << "Creating IioAdaptor with id: " << id;
    setup();
}

IioAdaptor::~IioAdaptor()
{
    if (accelerometerBuffer_)
        delete accelerometerBuffer_;
    if (gyroscopeBuffer_)
        delete gyroscopeBuffer_;
    if (alsBuffer_)
        delete alsBuffer_;
    if (magnetometerBuffer_)
        delete magnetometerBuffer_;
    if (proximityBuffer_)
        delete proximityBuffer_;
}

void IioAdaptor::setup()
{
    qDebug() << Q_FUNC_INFO << deviceId;

    if (deviceId.startsWith("accel")) {
        const QString sensorTypeInConfig = "accelerometer";
        const QString inputMatch = SensorFrameworkConfig::configuration()->value<QString>(sensorTypeInConfig + "/input_match");
        qDebug() << sensorTypeInConfig + ":" << "input_match" << inputMatch;

        iioDevice.channelTypeName = "accel";
        devNodeNumber = findSensor(inputMatch);
        if (devNodeNumber!= -1) {
            const QString description = "Industrial I/O accelerometer (" + iioDevice.name +")";

            accelerometerBuffer_ = new DeviceAdaptorRingBuffer<TimedXyzData>(1);
            setAdaptedSensor(sensorTypeInConfig, iioDevice.name, accelerometerBuffer_);
            setDescription(description);

            iioDevice.sensorType = IioAdaptor::IIO_ACCELEROMETER;
        }
    } else if (deviceId.startsWith("gyro")) {
        const QString sensorTypeInConfig = "gyroscope";
        const QString inputMatch = SensorFrameworkConfig::configuration()->value<QString>(sensorTypeInConfig + "/input_match");
        qDebug() << sensorTypeInConfig + ":" << "input_match" << inputMatch;

        iioDevice.channelTypeName = "anglvel";
        devNodeNumber = findSensor(inputMatch);
        if (devNodeNumber!= -1) {
            const QString description = "Industrial I/O gyroscope (" + iioDevice.name +")";
            gyroscopeBuffer_ = new DeviceAdaptorRingBuffer<TimedXyzData>(1);
            setAdaptedSensor(sensorTypeInConfig, iioDevice.name, gyroscopeBuffer_);
            setDescription(description);
            iioDevice.sensorType = IioAdaptor::IIO_GYROSCOPE;
        }
    } else if (deviceId.startsWith("mag")) {
        const QString sensorTypeInConfig = "magnetometer";
        const QString inputMatch = SensorFrameworkConfig::configuration()->value<QString>(sensorTypeInConfig + "/input_match");
        qDebug() << sensorTypeInConfig + ":" << "input_match" << inputMatch;

        iioDevice.channelTypeName = "magn";
        devNodeNumber = findSensor(inputMatch);
        if (devNodeNumber!= -1) {
            const QString description = "Industrial I/O magnetometer (" + iioDevice.name +")";

            magnetometerBuffer_ = new DeviceAdaptorRingBuffer<CalibratedMagneticFieldData>(1);
            setAdaptedSensor(sensorTypeInConfig, iioDevice.name, magnetometerBuffer_);
            setDescription(description);
            iioDevice.sensorType = IioAdaptor::IIO_MAGNETOMETER;
        }
    } else if (deviceId.startsWith("als")) {
        const QString sensorTypeInConfig = "als";
        const QString inputMatch = SensorFrameworkConfig::configuration()->value<QString>(sensorTypeInConfig + "/input_match");
        qDebug() << sensorTypeInConfig + ":" << "input_match" << inputMatch;

        iioDevice.channelTypeName = "illuminance";
        devNodeNumber = findSensor(inputMatch);
        if (devNodeNumber!= -1) {
            const QString description = "Industrial I/O light sensor (" + iioDevice.name +")";

            alsBuffer_ = new DeviceAdaptorRingBuffer<TimedUnsigned>(1);
            setAdaptedSensor(sensorTypeInConfig, iioDevice.name, alsBuffer_);
            setDescription(description);
            iioDevice.sensorType = IioAdaptor::IIO_ALS;
        }
    } else if (deviceId.startsWith("prox")) {
        const QString name = "proximity";
        const QString inputMatch = SensorFrameworkConfig::configuration()->value<QString>(name + "/input_match");
        qDebug() << name + ":" << "input_match" << inputMatch;

        iioDevice.channelTypeName = "proximity";
        devNodeNumber = findSensor(inputMatch);
        proximityThreshold = SensorFrameworkConfig::configuration()->value<QString>(name + "/threshold", QString(PROXIMITY_DEFAULT_THRESHOLD)).toInt();
        if (devNodeNumber!= -1) {
            QString description = "Industrial I/O proximity sensor (" + iioDevice.name +")";
            proximityBuffer_ = new DeviceAdaptorRingBuffer<ProximityData>(1);
            setAdaptedSensor(name, iioDevice.name, proximityBuffer_);
            setDescription(description);
            iioDevice.sensorType = IioAdaptor::IIO_PROXIMITY;
        }
    }

    if (devNodeNumber == -1) {
        qDebug() << Q_FUNC_INFO << "sensor is invalid";
//        setValid(false);
        return;
    }

    if (mode() != SysfsAdaptor::IntervalMode) {
        scanElementsEnable(devNodeNumber,1);
        scanElementsEnable(devNodeNumber,0);
    }

    /* Temperary switch this of. scale_override is always set which can not be true???? Loading issue deviceinfo???
    // Override the scaling factor if asked 
    bool ok;
    double scale_override = SensorFrameworkConfig::configuration()->value(iioDevice.name + "/scale").toDouble(&ok);
    if (ok) {
        sensordLogD() << "Overriding scale to" << scale_override;
        iioDevice.scale = scale_override;
    }
    */

    introduceAvailableDataRange(DataRange(0, 65535, 1));
    introduceAvailableInterval(DataRange(0, 586, 0));
    setDefaultInterval(10);
}

int IioAdaptor::findSensor(const QString &sensorName)
{
    udev_list_entry *devices;
    udev_list_entry *dev_list_entry;
    udev_device *dev = 0;
    struct udev *udevice = 0;
    struct udev_enumerate *enumerate = 0;

    if (!udevice)
        udevice = udev_new();

    enumerate = udev_enumerate_new(udevice);
    udev_enumerate_add_match_subsystem(enumerate, "iio");

    udev_enumerate_scan_devices(enumerate);
    devices = udev_enumerate_get_list_entry(enumerate);

    bool ok2;

    udev_list_entry_foreach(dev_list_entry, devices) {
        const char *path;
        path = udev_list_entry_get_name(dev_list_entry);

        dev = udev_device_new_from_syspath(udevice, path);
        if (qstrcmp(udev_device_get_subsystem(dev), "iio") == 0) {
            iioDevice.name = QString::fromLatin1(udev_device_get_sysattr_value(dev,"name"));
            if (iioDevice.name == sensorName) {
                struct udev_list_entry *sysattr;
                int j = 0;
                QString eventName = QString::fromLatin1(udev_device_get_sysname(dev));
                iioDevice.devicePath = QString::fromLatin1(udev_device_get_syspath(dev)) +"/";
                iioDevice.index = eventName.right(1).toInt(&ok2);
                // Default values
                iioDevice.offset = 0.0;
                iioDevice.scale = 1.0;
                iioDevice.frequency = 1.0;
                qDebug() << Q_FUNC_INFO << "Syspath for sensor (" + sensorName + "):" << iioDevice.devicePath;

                udev_list_entry_foreach(sysattr, udev_device_get_sysattr_list_entry(dev)) {
                    const char *name;
                    const char *value;
                    bool ok;
                    name = udev_list_entry_get_name(sysattr);
                    value = udev_device_get_sysattr_value(dev, name);
                    if (value == NULL)
                        continue;
                    qDebug() << "attr" << name << value;

                    QString attributeName(name);
                    if (attributeName.contains(QRegularExpression(iioDevice.channelTypeName + ".*scale$"))) {
                        iioDevice.scale = QString(value).toDouble(&ok);
                        if (ok) {
                            qDebug() << sensorName + ":" << "Scale is" << iioDevice.scale;
                        }
                    } else if (attributeName.contains(QRegularExpression(iioDevice.channelTypeName + ".*offset$"))) {
                        iioDevice.offset = QString(value).toDouble(&ok);
                        if (ok) {
                            qDebug() << sensorName + ":" << "Offset is" << value;
                        }
                    } else if (attributeName.endsWith("frequency")) {
                        iioDevice.frequency = QString(value).toDouble(&ok);
                        if (ok) {
                            qDebug() << sensorName + ":" << "Frequency is" << iioDevice.frequency;
                        }
                    } else if (attributeName.contains(QRegularExpression(iioDevice.channelTypeName + ".*raw$"))) {
                        qDebug() << "adding to paths:" << iioDevice.devicePath
                                   << attributeName << iioDevice.index;
                        addPath(iioDevice.devicePath + attributeName, j);
                        j++;
                    }
                }
                iioDevice.channels = j;

    // in_rot_from_north_magnetic_tilt_comp_raw ?

                // type
                break;
            }
        }
    }
    if (dev)
        udev_device_unref(dev);
    udev_enumerate_unref(enumerate);

    if (ok2)
        return iioDevice.index;
    else
        return -1;
}
/*
 * als
 * accel_3d
 * gyro_3d
 * magn_3d
 * incli_3d
 * dev_rotation
 *
 * */

bool IioAdaptor::deviceEnable(int device, int enable)
{
    qDebug() << Q_FUNC_INFO <<"device"<< device <<"enable" << enable;
    qDebug() << "devicePath" << iioDevice.devicePath << iioDevice.name;
    qDebug() << "dev_accl_" << devNodeNumber;
    qDebug() << "scale" << (double)iioDevice.scale
             << "offset" << iioDevice.offset
             << "frequency" << iioDevice.frequency;

    if (devNodeNumber == -1)
        return false;

    QString pathEnable = iioDevice.devicePath + "buffer/enable";
    QString pathLength = iioDevice.devicePath + "buffer/length";

    qDebug() << pathEnable << pathLength;

    if (enable == 1) {
        // FIXME: should enable sensors for this device? Assuming enabled already
        scanElementsEnable(device, enable);
        sysfsWriteInt(pathLength, IIO_BUFFER_LEN);
        sysfsWriteInt(pathEnable, enable);
    } else {
        sysfsWriteInt(pathEnable, enable);
        scanElementsEnable(device, enable);
        // FIXME: should disable sensors for this device?
    }

    return true;
}

bool IioAdaptor::sysfsWriteInt(QString filename, int val)
{
    QFile file(filename);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        sensordLogW() << "Failed to open " << filename;
        return false;
    }

    QTextStream out(&file);
    out << val << "\n";

    file.close();

	return true;
}

QString IioAdaptor::sysfsReadString(QString filename)
{
    QFile file(filename);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        sensordLogW() << "Failed to open " << filename;
        return QString();
    }

    QTextStream in(&file);
    QString line = in.readLine();

    if (line.endsWith("\n")) {
        line.chop(1);
    }

    file.close();
    return line;
}

int IioAdaptor::sysfsReadInt(QString filename)
{
    QString string = sysfsReadString(filename);

    bool ok;
    int value = string.toInt(&ok);

    if (!ok) {
        sensordLogW() << "Failed to parse '" << string << "' to int from file " << filename;
    }

	return value;
}

// Return the number of channels
int IioAdaptor::scanElementsEnable(int device, int enable)
{
    Q_UNUSED(device);

    QString elementsPath = iioDevice.devicePath + "scan_elements";

    QDir dir(elementsPath);
    if (!dir.exists()) {
        sensordLogW() << "Directory " << elementsPath << " doesn't exist";
        return 0;
    }

    // Find all the *_en file and write 0/1 to it
    QStringList filters;
    filters << ("*" + iioDevice.channelTypeName + "*_en");
    dir.setNameFilters(filters);

    QFileInfoList list = dir.entryInfoList();
    for (int i = 0; i < list.size(); ++i) {
        QFileInfo fileInfo = list.at(i);

        if (enable) {
            QString base = fileInfo.filePath();
            // Remove the _en
            base.chop(3);

            int index = sysfsReadInt(base + "_index");
            int bytes = deviceChannelParseBytes(base + "_type");

           iioDevice.channel_bytes[index] = bytes;
        }

        sysfsWriteInt(fileInfo.filePath(), enable);
    }

    return list.size();
}

int IioAdaptor::deviceChannelParseBytes(QString filename)
{
    QString type = sysfsReadString(filename);

    if (type.compare("le:s16/16>>0") == 0) {
        return 2;
    } else if (type.compare("le:s32/32>>0") == 0) {
        return 4;
    } else if (type.compare("le:s64/64>>0") == 0) {
        return 8;
    } else {
        sensordLogW() << "ERROR: invalid type from file " << filename << ": " << type;
    }

    return 0;
}

void IioAdaptor::processSample(int fileId, int fd)
{
    char buf[IIO_BUFFER_LEN];
    int readBytes = 0;
    qreal result = 0;
    int channel = fileId%IIO_MAX_DEVICE_CHANNELS;
    int device = (fileId - channel)/IIO_MAX_DEVICE_CHANNELS;

    if (device == 0) {
        readBytes = read(fd, buf, sizeof(buf));

        if (readBytes <= 0) {
            sensordLogW() << "read():" << strerror(errno);
            return;
        }

        errno = 0; // reset errno before call
        result = strtol(buf, NULL, 10);
        
        // If any conversion error occurs, abort
        if (errno != 0) {
            sensordLogW() << "strtol(): Unable to convert string to long"; 
            return;
        }

        switch(channel) {
        case 0: {
            switch (iioDevice.sensorType) {
            case IioAdaptor::IIO_ACCELEROMETER:
                accelData = accelerometerBuffer_->nextSlot();
                // sensorfw works with milli-G. Hence converting m/s^2 to milli-G. See link (1)
                // Hardware calibration offset in_accel_x_calibbias (assumed to fix productioninaccuracies) is already applied.
                accelData->x_= -(result + iioDevice.offset) * iioDevice.scale * 1000 * REV_GRAVITY;
                break;
            case IioAdaptor::IIO_GYROSCOPE:
                gyroData = gyroscopeBuffer_->nextSlot();
                // sensorfw works with milidegrees/s. Hence converting rad/s -> milidegrees/s. See link (1)
                // Hardware calibration offset in_anglvel_x_calibbias (assumed to fix productioninaccuracies) is already applied.
                gyroData->x_= (result + iioDevice.offset) * iioDevice.scale  * 1000 * RADIANS_TO_DEGREES;
                break;
            case IioAdaptor::IIO_MAGNETOMETER:
                calMagData = magnetometerBuffer_->nextSlot();
                // sensorfw works with nano Tesla. Hence converting Gaus -> nT. See link (1)
                // inserting the raw x data (rx_)
                calMagData->rx_ = (result + iioDevice.offset) * iioDevice.scale * 100000;
                break;
            case IioAdaptor::IIO_ALS:
                uData = alsBuffer_->nextSlot();
                uData->value_ = (result + iioDevice.offset) * iioDevice.scale;
                break;
            case IioAdaptor::IIO_PROXIMITY:
                {
                    bool near = false;
                    int proximityValue = (result + iioDevice.offset) * iioDevice.scale;
                    proximityData = proximityBuffer_->nextSlot();
                    // IIO proximity sensors are inverted in comparison to Hybris proximity sensors
                    if (proximityValue >= proximityThreshold) {
                        near = true;
                    }
                    proximityData->withinProximity_ = near;
                    proximityData->value_ = near ? PROXIMITY_NEAR_VALUE : PROXIMITY_FAR_VALUE;
                }
                break;
            default:
                break;
            };
        }
            break;

        case 1: {
            switch (iioDevice.sensorType) {
            case IioAdaptor::IIO_ACCELEROMETER:
                accelData = accelerometerBuffer_->nextSlot();
                accelData->y_= -(result + iioDevice.offset) * iioDevice.scale * 1000 * REV_GRAVITY;
                break;
            case IioAdaptor::IIO_GYROSCOPE:
                gyroData = gyroscopeBuffer_->nextSlot();
                gyroData->y_= (result + iioDevice.offset) * iioDevice.scale * 1000* RADIANS_TO_DEGREES;
                break;
            case IioAdaptor::IIO_MAGNETOMETER:
                calMagData = magnetometerBuffer_->nextSlot();
                calMagData->ry_ = (result + iioDevice.offset) * iioDevice.scale * 100000;
                break;
            default:
                break;
            };
        }
            break;

        case 2: {
            switch (iioDevice.sensorType) {
            case IioAdaptor::IIO_ACCELEROMETER:
                accelData = accelerometerBuffer_->nextSlot();
                accelData->z_ = -(result + iioDevice.offset) * iioDevice.scale * 1000 * REV_GRAVITY;
                break;
            case IioAdaptor::IIO_GYROSCOPE:
                gyroData = gyroscopeBuffer_->nextSlot();
                gyroData->z_ = (result + iioDevice.offset) * iioDevice.scale * 1000 * RADIANS_TO_DEGREES;
                break;
            case IioAdaptor::IIO_MAGNETOMETER:
                calMagData = magnetometerBuffer_->nextSlot();
                calMagData->rz_ = (result + iioDevice.offset) * iioDevice.scale * 100000;
                break;
            default:
                break;
            };
        }
            break;
        };

        if (channel == iioDevice.channels - 1) {
            switch (iioDevice.sensorType) {
            case IioAdaptor::IIO_ACCELEROMETER:
                accelData->timestamp_ = Utils::getTimeStamp();
                accelerometerBuffer_->commit();
                accelerometerBuffer_->wakeUpReaders();
                // Uncomment line to test accelerometer sensor
                //sensordLogT() << "Accelerometer offset=" << iioDevice.offset << "scale=" << iioDevice.scale << "x=" << accelData->x_ << "y=" << accelData->y_ << "z=" << accelData->z_ << "timestamp=" << accelData->timestamp_;
                break;
            case IioAdaptor::IIO_GYROSCOPE:
                gyroData->timestamp_ = Utils::getTimeStamp();
                gyroscopeBuffer_->commit();
                gyroscopeBuffer_->wakeUpReaders();
                //sensordLogT() << "Gyroscope offset=" << iioDevice.offset << "scale=" << iioDevice.scale << "x=" << gyroData->x_ << "y=" << gyroData->y_ << "z=" << gyroData->z_ << "timestamp=" << gyroData->timestamp_;
                break;
            case IioAdaptor::IIO_MAGNETOMETER:
                calMagData->timestamp_ = Utils::getTimeStamp();
                magnetometerBuffer_->commit();
                magnetometerBuffer_->wakeUpReaders();
                // Uncomment line to test magnetometer sensor
                sensordLogT() << "Magnetometer offset=" << iioDevice.offset << "scale=" << iioDevice.scale << "x=" << calMagData->rx_ << "y=" << calMagData->ry_ << calMagData->rz_ << "timestamp=" << calMagData->timestamp_;
                break;
            case IioAdaptor::IIO_ALS:
                uData->timestamp_ = Utils::getTimeStamp();
                alsBuffer_->commit();
                alsBuffer_->wakeUpReaders();
                // Uncomment line to test ALS sensor
                //sensordLogT() << "ALS offset=" << iioDevice.offset << "scale=" << iioDevice.scale << "value=" << uData->value_ << "timestamp=" << uData->timestamp_;
                break;
            case IioAdaptor::IIO_PROXIMITY:
                proximityData->timestamp_ = Utils::getTimeStamp();
                proximityBuffer_->commit();
                proximityBuffer_->wakeUpReaders();
                // Uncomment line to test proximity sensor
                //sensordLogT() << "Proximity offset=" << iioDevice.offset << "scale=" << iioDevice.scale << "value=" << proximityData->value_ << "within proximity=" << proximityData->withinProximity_ << "timestamp=" << proximityData->timestamp_;
                break;
            default:
                break;
            };
        }
    }
}

bool IioAdaptor::setInterval(const unsigned int value, const int sessionId)
{
    if (mode() == SysfsAdaptor::IntervalMode)
        return SysfsAdaptor::setInterval(value, sessionId);

    sensordLogD() << "Ignoring setInterval for " << value;

    return true;
}

//unsigned int IioAdaptor::interval() const
//{
//    int value = 100;
//    sensordLogD() << "Returning dummy value in interval(): " << value;
//    return value;
//}


bool IioAdaptor::startSensor()
{
    if (devNodeNumber == -1)
        return false;

    qDebug() << Q_FUNC_INFO;
    if (mode() != SysfsAdaptor::IntervalMode)
        deviceEnable(devNodeNumber, true);
    return SysfsAdaptor::startSensor();
}

void IioAdaptor::stopSensor()
{
    if (devNodeNumber == -1)
        return;
    qDebug() << Q_FUNC_INFO;
    if (mode() != SysfsAdaptor::IntervalMode)
        deviceEnable(devNodeNumber, false);
    SysfsAdaptor::stopSensor();
}
