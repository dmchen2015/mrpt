/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "obs-precomp.h"  // Precompiled headers

#include <mrpt/serialization/CArchive.h>
#include <mrpt/obs/CObservationObject.h>
#include <mrpt/system/os.h>
#include <mrpt/math/matrix_serialization.h>  // for << ops
#include <mrpt/math/wrap2pi.h>
#include <set>
#include <mrpt/poses/CPose2D.h>

#include <iostream>

using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace mrpt::math;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationObject, CObservation, mrpt::obs)

/*---------------------------------------------------------------
 Default constructor.
 ---------------------------------------------------------------*/
CObservationObject::CObservationObject()
	: minSensorDistance(0),
	  maxSensorDistance(0),
	  fieldOfView_yaw(DEG2RAD(180)),
	  fieldOfView_pitch(DEG2RAD(90)),
	  sensorLocationOnRobot(),
	  sensedData(),
	  validCovariances(false),
	  sensor_std_range(0),
	  sensor_std_yaw(0),
	  sensor_std_pitch(0)
{
}

uint8_t CObservationObject::serializeGetVersion() const { return 3; }

void CObservationObject::serializeTo(
	mrpt::serialization::CArchive& out) const
{
	uint32_t i, n;
	// The data
	out << minSensorDistance << maxSensorDistance << fieldOfView_yaw
		<< fieldOfView_pitch << sensorLocationOnRobot << timestamp;
	out << validCovariances;
	if (!validCovariances)
		out << sensor_std_range << sensor_std_yaw << sensor_std_pitch;

	// Detect duplicate landmarks ID, which is an error!
	std::set<int32_t> lstIDs;

	n = sensedData.size();
	out << n;
	for (i = 0; i < n; i++)
	{
		int32_t id = sensedData[i].landmarkID;
		if (id != INVALID_LANDMARK_ID)
		{
			if (0 != lstIDs.count(id))
				THROW_EXCEPTION_FMT("Duplicate landmark ID=%i found.", (int)id);
			lstIDs.insert(id);
		}

		out << sensedData[i].range << sensedData[i].yaw << sensedData[i].pitch << id;
 	    out << sensedData[i].pose_wo << sensedData[i].pose_so << sensedData[i].shape_variables;

		if (validCovariances) out << sensedData[i].covariance;
	}

	out << sensorLabel;
}



void CObservationObject::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		case 1:
		case 2:
		case 3:
		{
			uint32_t i, n;

			// The data
			in >> minSensorDistance >> maxSensorDistance;

			if (version >= 3)
			{
				in >> fieldOfView_yaw >> fieldOfView_pitch;
			}
			else
			{
				float fieldOfView;
				in >> fieldOfView;

				fieldOfView_yaw = fieldOfView_pitch = fieldOfView;
			}

			in >> sensorLocationOnRobot;

			if (version >= 2)
				in >> timestamp;
			else
				timestamp = INVALID_TIMESTAMP;

			if (version >= 3)
			{
				in >> validCovariances;
				if (!validCovariances)
					in >> sensor_std_range >> sensor_std_yaw >>
						sensor_std_pitch;
			}
			else
				validCovariances = false;

			in >> n;
			sensedData.resize(n);

			// Detect duplicate landmarks ID, what is an error!
			std::set<int32_t> lstIDs;

			for (i = 0; i < n; i++)
			{
				in >> sensedData[i].range >> sensedData[i].yaw >>
          			  sensedData[i].pitch >> sensedData[i].landmarkID;
        		
                in >> sensedData[i].pose_wo >> sensedData[i].pose_so >> sensedData[i].shape_variables;

				if (version >= 3 && validCovariances)
					in >> sensedData[i].covariance;

				int32_t id = sensedData[i].landmarkID;
				if (id != INVALID_LANDMARK_ID)
				{
					if (0 != lstIDs.count(id))
						THROW_EXCEPTION_FMT(
							"Duplicate landmark ID=%i found.", (int)id);
					lstIDs.insert(id);
				}
			}

			if (version >= 1)
				in >> sensorLabel;
			else
				sensorLabel = "";
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

void CObservationObject::getMeasurementAsPose3DVector(std::vector<mrpt::poses::CPose3D> &pose, bool robot_space) const
{
    pose.resize(sensedData.size());
    int c=0;
    for(CObservationObject::const_iterator it=sensedData.begin();
                    it!=sensedData.end(); ++it, ++c)
    {
        double pitch = it->pitch;
        double yaw = it->yaw;
        double R = it->range;
        CPose2D p2d;
        p2d.x(R * cos(yaw));
        p2d.y(R * sin(yaw));
        p2d.phi(yaw);
        pose[c] = CPose3D(p2d);
    }
}

void CObservationObject::debugPrintOut()
{
  printf("[CObservationObject::debugPrintOut] Dumping:\n");
	printf(
    "[CObservationObject::debugPrintOut] minSensorDistance:\t%f\n",
		minSensorDistance);
	printf(
    "[CObservationObject::debugPrintOut] maxSensorDistance:\t%f:\n",
		maxSensorDistance);
	printf(
    "[CObservationObject::debugPrintOut] %u landmarks:\n",
		static_cast<unsigned>(sensedData.size()));

	size_t i, n = sensedData.size();
	for (i = 0; i < n; i++)
		printf(
      "[CObservationObject::debugPrintOut] \tID[%i]: y:%fdeg "
			"p:%fdeg range: %f\n",
			sensedData[i].landmarkID, RAD2DEG(sensedData[i].yaw),
			RAD2DEG(sensedData[i].pitch), sensedData[i].range);
}

void CObservationObject::getDescriptionAsText(std::ostream& o) const
{
	using namespace std;
	CObservation::getDescriptionAsText(o);

	o << "Homogeneous matrix for the sensor's 3D pose, relative to robot "
		 "base:\n";
	o << sensorLocationOnRobot.getHomogeneousMatrixVal<CMatrixDouble44>()
	  << sensorLocationOnRobot << endl
	  << endl;

	o << "Do observations have individual covariance matrices? "
	  << (validCovariances ? "YES" : "NO") << endl
	  << endl;

	o << "Default noise sigmas:" << endl;
	o << "sensor_std_range (m)   : " << sensor_std_range << endl;
	o << "sensor_std_yaw   (deg) : " << RAD2DEG(sensor_std_yaw) << endl;
	o << "sensor_std_pitch (deg) : " << RAD2DEG(sensor_std_pitch) << endl;

	o << endl;

	// For each entry in this sequence:
	o << "  LANDMARK_ID    RANGE (m)    YAW (deg)    PITCH (deg)   COV. MATRIX "
		 "(optional)"
	  << endl;
	o << "---------------------------------------------------------------------"
		 "-----------------"
	  << endl;
	for (size_t q = 0; q < sensedData.size(); q++)
	{
		o << "      ";
		if (sensedData[q].landmarkID == INVALID_LANDMARK_ID)
			o << "(NO ID)";
		else
			o << format("%7u", sensedData[q].landmarkID);

		o << format(
			"   %10.03f  %10.03f %10.03f        ", sensedData[q].range,
			RAD2DEG(mrpt::math::wrapToPi(sensedData[q].yaw)),
			RAD2DEG(mrpt::math::wrapToPi(sensedData[q].pitch)));

		if (validCovariances)
			o << sensedData[q].covariance.inMatlabFormat() << endl;
		else
			o << "  (N/A)\n";
	}
}
