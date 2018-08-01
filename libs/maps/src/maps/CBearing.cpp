/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "maps-precomp.h"  // Precomp header

#include <mrpt/maps/CBearing.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/serialization/CArchive.h>

#include <mrpt/system/os.h>
#include <mrpt/math/geometry.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/CEllipsoid.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CText.h>
#include <mrpt/opengl/CSphere.h>

using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace mrpt::poses;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CBearing, CSerializable, mrpt::maps)

uint8_t CBearing::serializeGetVersion() const { return 0; }
void CBearing::serializeTo(mrpt::serialization::CArchive& out) const
{
	uint32_t i = m_ID;
	uint32_t j = m_typePDF;
    out << i << j << m_locationMC << m_locationGauss << m_locationSOG << m_locationNoPDF;
}

void CBearing::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		{
			uint32_t i, j;
            in >> i >> j >> m_locationMC >> m_locationGauss >> m_locationSOG >> m_locationNoPDF;
			m_ID = i;
			m_typePDF = static_cast<TTypePDF>(j);
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

/*---------------------------------------------------------------
					getMean
  ---------------------------------------------------------------*/
void CBearing::getMean(CPose3D& p) const
{
	MRPT_START
	switch (m_typePDF)
	{
		case pdfMonteCarlo:
			m_locationMC.getMean(p);
			break;
		case pdfGauss:
			m_locationGauss.getMean(p);
			break;
		case pdfSOG:
			m_locationSOG.getMean(p);
			break;
        case pdfNO:
            m_locationNoPDF.getMean(p);
            break;
		default:
			THROW_EXCEPTION("ERROR: Invalid 'm_typePDF' value");
	};
	MRPT_END
}

/*---------------------------------------------------------------
					getCovarianceAndMean
  ---------------------------------------------------------------*/
void CBearing::getCovarianceAndMean(CMatrixDouble66 &COV, CPose3D& p) const
{
	MRPT_START
	switch (m_typePDF)
	{
		case pdfMonteCarlo:
			m_locationMC.getCovarianceAndMean(COV, p);
			break;
		case pdfGauss:
			m_locationGauss.getCovarianceAndMean(COV, p);
			break;
		case pdfSOG:
			m_locationSOG.getCovarianceAndMean(COV, p);
			break;
        case pdfNO:
            m_locationNoPDF.getCovarianceAndMean(COV, p);
            break;
		default:
			THROW_EXCEPTION("ERROR: Invalid 'm_typePDF' value");
	};
	MRPT_END
}

/*---------------------------------------------------------------
					bayesianFusion
  ---------------------------------------------------------------*/
void CBearing::bayesianFusion(const CPose3DPDF& p1, const CPose3DPDF& p2)
{
	MRPT_START
	switch (m_typePDF)
	{
		case pdfMonteCarlo:
            m_locationMC.bayesianFusion(p1, p2);
			break;
		case pdfGauss:
            m_locationGauss.bayesianFusion(p1, p2);
			break;
		case pdfSOG:
            m_locationSOG.bayesianFusion(p1, p2);
			break;
        case pdfNO:
            m_locationNoPDF.bayesianFusion(p1, p2);
            break;
		default:
			THROW_EXCEPTION("ERROR: Invalid 'm_typePDF' value");
	};
	MRPT_END
}

/*---------------------------------------------------------------
                    inverse
  ---------------------------------------------------------------*/
void CBearing::inverse(CPose3DPDF &o) const
{
    MRPT_START
    switch (m_typePDF)
    {
        case pdfMonteCarlo:
            m_locationMC.inverse(o);
            break;
        case pdfGauss:
            m_locationGauss.inverse(o);
            break;
        case pdfSOG:
            m_locationSOG.inverse(o);
            break;
        case pdfNO:
            m_locationNoPDF.inverse(o);
            break;
        default:
            THROW_EXCEPTION("Error: Invalid 'm_typePDF' value");
    }
    MRPT_END
}

/*---------------------------------------------------------------
					drawSingleSample
  ---------------------------------------------------------------*/
void CBearing::drawSingleSample(CPose3D& outSample) const
{
	MRPT_START
	switch (m_typePDF)
	{
		case pdfMonteCarlo:
			m_locationMC.drawSingleSample(outSample);
			break;
		case pdfGauss:
			m_locationGauss.drawSingleSample(outSample);
			break;
		case pdfSOG:
			m_locationSOG.drawSingleSample(outSample);
			break;
        case pdfNO:
            m_locationNoPDF.drawSingleSample(outSample);
            break;
		default:
			THROW_EXCEPTION("ERROR: Invalid 'm_typePDF' value");
	};
	MRPT_END
}

/*---------------------------------------------------------------
					copyFrom
  ---------------------------------------------------------------*/
void CBearing::copyFrom(const CPose3DPDF& o)
{
	MRPT_START
	switch (m_typePDF)
	{
		case pdfMonteCarlo:
			m_locationMC.copyFrom(o);
			break;
		case pdfGauss:
			m_locationGauss.copyFrom(o);
			break;
		case pdfSOG:
			m_locationSOG.copyFrom(o);
			break;
        case pdfNO:
            m_locationNoPDF.copyFrom(o);
            break;
		default:
			THROW_EXCEPTION("ERROR: Invalid 'm_typePDF' value");
	};
	MRPT_END
}

/*---------------------------------------------------------------
					saveToTextFile
  ---------------------------------------------------------------*/
bool CBearing::saveToTextFile(const std::string& file) const
{
	MRPT_START
	switch (m_typePDF)
	{
		case pdfMonteCarlo:
			return m_locationMC.saveToTextFile(file);
			break;
		case pdfGauss:
			return m_locationGauss.saveToTextFile(file);
			break;
		case pdfSOG:
			return m_locationSOG.saveToTextFile(file);
			break;
        case pdfNO:
            return m_locationNoPDF.saveToTextFile(file);
            break;
		default:
			THROW_EXCEPTION("ERROR: Invalid 'm_typePDF' value");
	};
	MRPT_END
}

/*---------------------------------------------------------------
					changeCoordinatesReference
  ---------------------------------------------------------------*/
void CBearing::changeCoordinatesReference(const CPose3D& newReferenceBase)
{
	MRPT_START
	switch (m_typePDF)
	{
		case pdfMonteCarlo:
			m_locationMC.changeCoordinatesReference(newReferenceBase);
			break;
		case pdfGauss:
			m_locationGauss.changeCoordinatesReference(newReferenceBase);
			break;
		case pdfSOG:
			m_locationSOG.changeCoordinatesReference(newReferenceBase);
			break;
        case pdfNO:
            m_locationNoPDF.changeCoordinatesReference(newReferenceBase);
            break;
		default:
			THROW_EXCEPTION("ERROR: Invalid 'm_typePDF' value");
	};
	MRPT_END
}

/*---------------------------------------------------------------
					getAs3DObject
  ---------------------------------------------------------------*/
void CBearing::getAs3DObject(mrpt::opengl::CSetOfObjects::Ptr& outObj) const
{
	MRPT_START
	switch (m_typePDF)
	{
		case pdfMonteCarlo:
		{
            opengl::CPointCloud::Ptr obj =
                mrpt::make_aligned_shared<opengl::CPointCloud>();
			obj->setColor(1, 0, 0);

			obj->setPointSize(2.5);

			const size_t N = m_locationMC.m_particles.size();
			obj->resize(N);

			for (size_t i = 0; i < N; i++)
				obj->setPoint(
                    i, m_locationMC.m_particles[i].d.x,
                    m_locationMC.m_particles[i].d.y,
                    m_locationMC.m_particles[i].d.z);

			outObj->insert(obj);
		}
		break;
		case pdfGauss:
		{
			opengl::CEllipsoid::Ptr obj =
				mrpt::make_aligned_shared<opengl::CEllipsoid>();

			obj->setPose(m_locationGauss.mean);
			obj->setLineWidth(3);

			CMatrixDouble C = CMatrixDouble(m_locationGauss.cov);
			if (C(2, 2) == 0) C.setSize(2, 2);
			obj->setCovMatrix(C);

			obj->setQuantiles(3);
			obj->enableDrawSolid3D(false);

			obj->setColor(1, 0, 0, 0.85);
			outObj->insert(obj);
		}
		break;
		case pdfSOG:
		{
			m_locationSOG.getAs3DObject(outObj);
		}
		break;
        case pdfNO:
        {
            opengl::CPointCloud::Ptr obj =
                mrpt::make_aligned_shared<opengl::CPointCloud>();
            obj->setColor(1, 0, 0);

            obj->setPointSize(2.5);

            const size_t N = m_locationNoPDF.m_particles.size();
            obj->resize(N);

            for (size_t i = 0; i < N; i++)
                obj->setPoint(
                    i, m_locationNoPDF.m_particles[i].d.x,
                    m_locationNoPDF.m_particles[i].d.y,
                    m_locationNoPDF.m_particles[i].d.z);

            outObj->insert(obj);
        }
        break;
		default:
			THROW_EXCEPTION("ERROR: Invalid 'm_typePDF' value");
	};

	opengl::CText::Ptr obj2 = mrpt::make_aligned_shared<opengl::CText>();
	obj2->setString(format("#%d", static_cast<int>(m_ID)));

    CPose3D meanP;
	this->getMean(meanP);
	obj2->setLocation(meanP.x() + 0.10, meanP.y() + 0.10, meanP.z());
	outObj->insert(obj2);

	MRPT_END
}

/*---------------------------------------------------------------
					getAsMatlabDrawCommands
  ---------------------------------------------------------------*/
void CBearing::getAsMatlabDrawCommands(std::vector<std::string>& out_Str) const
{
	MRPT_START

	out_Str.clear();
	char auxStr[1000];

	switch (m_typePDF)
	{
		case pdfMonteCarlo:
		{
			// xs=[...];
			// ys=[...];
			// plot(xs,ys,'.','MarkerSize',4);
			size_t i, N = m_locationMC.m_particles.size();
			std::string sx, sy;

			sx = "xs=[";
			sy = "ys=[";
			for (i = 0; i < N; i++)
			{
				os::sprintf(
					auxStr, sizeof(auxStr), "%.3f%c",
                    m_locationMC.m_particles[i].d.x, (i == N - 1) ? ' ' : ',');
				sx = sx + std::string(auxStr);
				os::sprintf(
					auxStr, sizeof(auxStr), "%.3f%c",
                    m_locationMC.m_particles[i].d.y, (i == N - 1) ? ' ' : ',');
				sy = sy + std::string(auxStr);
			}
			sx = sx + "];";
			sy = sy + "];";
			out_Str.emplace_back(sx);
			out_Str.emplace_back(sy);
			out_Str.emplace_back("plot(xs,ys,'k.','MarkerSize',4);");
		}
		break;
		case pdfGauss:
		{
			// m=[x y];
			// C=[2x2]
			// error_ellipse(C,m,'conf',0.997,'style','k');

			os::sprintf(
				auxStr, sizeof(auxStr), "m=[%.3f %.3f];",
				m_locationGauss.mean.x(), m_locationGauss.mean.y());
			out_Str.emplace_back(auxStr);
			os::sprintf(
				auxStr, sizeof(auxStr), "C=[%e %e;%e %e];",
				m_locationGauss.cov(0, 0), m_locationGauss.cov(0, 1),
				m_locationGauss.cov(1, 0), m_locationGauss.cov(1, 1));
			out_Str.emplace_back(auxStr);

			out_Str.emplace_back(
				"error_ellipse(C,m,'conf',0.997,'style','k');");
		}
		break;
		case pdfSOG:
		{
            for (CPose3DPDFSOG::const_iterator it = m_locationSOG.begin();
				 it != m_locationSOG.end(); ++it)
			{
				os::sprintf(
					auxStr, sizeof(auxStr), "m=[%.3f %.3f];",
					(it)->val.mean.x(), (it)->val.mean.y());
				out_Str.emplace_back(auxStr);
				os::sprintf(
					auxStr, sizeof(auxStr), "C=[%e %e;%e %e];",
					(it)->val.cov(0, 0), (it)->val.cov(0, 1),
					(it)->val.cov(1, 0), (it)->val.cov(1, 1));
				out_Str.emplace_back(auxStr);
				out_Str.emplace_back(
					"error_ellipse(C,m,'conf',0.997,'style','k');");
			}
		}
		break;
        case pdfNO:
        {
            size_t i, N = m_locationNoPDF.size();
            std::string sx, sy;

            sx = "xs=[";
            sy = "ys=[";
            for (i = 0; i < N; i++)
            {
                os::sprintf(
                    auxStr, sizeof(auxStr), "%.3f%c",
                    m_locationNoPDF.m_particles[i].d.x, (i == N - 1) ? ' ' : ',');
                sx = sx + std::string(auxStr);
                os::sprintf(
                    auxStr, sizeof(auxStr), "%.3f%c",
                    m_locationNoPDF.m_particles[i].d.y, (i == N - 1) ? ' ' : ',');
                sy = sy + std::string(auxStr);
            }
            sx = sx + "];";
            sy = sy + "];";
            out_Str.emplace_back(sx);
            out_Str.emplace_back(sy);
            out_Str.emplace_back("plot(xs,ys,'k.','MarkerSize',4);");
        }
        break;
		default:
			THROW_EXCEPTION("ERROR: Invalid 'm_typePDF' value");
	};

	// The text:
    CPose3D meanP;
	getMean(meanP);

	os::sprintf(
		auxStr, sizeof(auxStr), "text(%f,%f,'#%i');", meanP.x(), meanP.y(),
		static_cast<int>(m_ID));
	out_Str.emplace_back(auxStr);

	MRPT_END
}

/*---------------------------------------------------------------
					generateObservationModelDistribution
 Compute the observation model p(z_t|x_t) for a given observation (range value),
and return it as an approximate SOG.
*  Note that if the beacon is a SOG itself, the number of gaussian modes will be
square.
*  As a speed-up, if a "center point"+"maxDistanceFromCenter" is supplied
(maxDistanceFromCenter!=0), those modes farther than this sphere will be
discarded.
*  Parameters such as the stdSigma of the sensor are gathered from "myBeaconMap"
*  The result is one "ring" for each Gaussian mode that represent the beacon
position in this object.
*  The position of the sensor on the robot is used to shift the resulting
densities such as they represent the position of the robot, not the sensor.
*  \sa CBearingMap::insertionOptions, generateRingSOG

  ---------------------------------------------------------------*/
void CBearing::generateObservationModelDistribution(const float& sensedRange, CPose3DPDFSOG& outPDF,
    const CBearingMap *myBearingMap, const CPose3D& sensorPntOnRobot,
    const CPose3D& centerPoint, const float& maxDistanceFromCenter) const
{
	MRPT_START

    const CPose3DPDFSOG* beaconPos = nullptr;

	if (m_typePDF == pdfGauss)
	{
		// Copy the gaussian to the SOG:
        CPose3DPDFSOG* new_beaconPos = new CPose3DPDFSOG(1);
        new_beaconPos->copyFrom(m_locationGauss);
		beaconPos = new_beaconPos;
	}
	else
	{
		ASSERT_(m_typePDF == pdfSOG);
        beaconPos = static_cast<const CPose3DPDFSOG*>(&m_locationSOG);
	}

	outPDF.clear();

    for (CPose3DPDFSOG::const_iterator it = beaconPos->begin();
		 it != beaconPos->end(); ++it)
	{
		// The center of the ring to be generated
        CPose3D ringCenter(
			(it)->val.mean.x() - sensorPntOnRobot.x(),
			(it)->val.mean.y() - sensorPntOnRobot.y(),
			(it)->val.mean.z() - sensorPntOnRobot.z());

		size_t startIdx = outPDF.size();

        CBearing::generateRingSOG(
			sensedRange,  // Sensed range
			outPDF,  // The ouput (Append all !!)
            myBearingMap,  // For params
			ringCenter,  // The center of the ring to be generated
			&(it)->val.cov,  // The covariance to ADD to each mode, due to the
			// composition of uncertainty
			false,  // clearPreviousContentsOutPDF
			centerPoint,
			maxDistanceFromCenter  // Directly, do not create too far modes
			);

		// Adjust the weights to the one of "this" mode:
        for (CPose3DPDFSOG::iterator itout = outPDF.begin(); it != outPDF.end(); ++it)
            (*itout).log_w = (it)->log_w;
	}

	if (m_typePDF == pdfGauss) delete beaconPos;

	MRPT_END
}

/*---------------------------------------------------------------
					generateRingSOG
  ---------------------------------------------------------------*/
void CBearing::generateRingSOG(const float& R, CPose3DPDFSOG& outPDF, const CBearingMap *myBearingMap,
    const mrpt::poses::CPose3D &sensorPnt,
    const CMatrixDouble66* covarianceCompositionToAdd,
    bool clearPreviousContentsOutPDF, const CPose3D& centerPoint,
    const float& maxDistanceFromCenter)
{
	MRPT_START

    ASSERT_(myBearingMap);

//	// Compute the number of Gaussians:
//	const float minEl = DEG2RAD(myBeaconMap->insertionOptions.minElevation_deg);
//	const float maxEl = DEG2RAD(myBeaconMap->insertionOptions.maxElevation_deg);
//	ASSERT_(
//		myBeaconMap->insertionOptions.minElevation_deg <=
//		myBeaconMap->insertionOptions.maxElevation_deg);

//	double el, th, A_ang;
//	const float maxDistBetweenGaussians =
//		myBeaconMap->insertionOptions.SOG_maxDistBetweenGaussians;  // Meters

//	// B: Number of gaussians in each cut to the sphere (XY,XZ,...)
//	size_t B = (size_t)(M_2PIf * R / maxDistBetweenGaussians) + 1;

//	// Assure minimum B (maximum angular increment):
//	B = max(B, (size_t)30);

//	// B must be even:
//	if (B % 2) B++;

//	A_ang = M_2PI / B;  // Angular increments between modes:

//	// The diagonal basic covariance matrix.
//	//  (0,0) is the variance in the direction "in->out" of the sphere
//	//  (1,1),(2,2) is the var. in the two directions tangent to the sphere
//	CMatrixDouble33 S;
//	S(0, 0) = square(myBeaconMap->likelihoodOptions.rangeStd);
//	S(1, 1) = S(2, 2) = square(
//		A_ang * R /
//		myBeaconMap->insertionOptions
//			.SOG_separationConstant);  // 4.0f * sensedRange * S(0,0);

//    CPose3D dir;

//	// Create the SOG:
//	size_t modeIdx;
//	if (clearPreviousContentsOutPDF)
//	{
//		// Overwrite modes:
//		modeIdx = 0;
//		outPDF.resize(B * B);
//	}
//	else
//	{
//		// Append modes:
//		modeIdx = outPDF.size();  // Start here
//		outPDF.resize(outPDF.size() + B * B);
//	}

//	size_t idxEl, idxTh;  // idxEl:[0,B/2+1]

//	for (idxEl = 0; idxEl <= (1 + B / 2); idxEl++)
//	{
//		el = minEl + idxEl * A_ang;
//		if (el > (maxEl + 0.5 * A_ang)) continue;

//		size_t nThSteps = B;
//		// Except if we are in the top/bottom of the sphere:
//		if (fabs(cos(el)) < 1e-4)
//		{
//			nThSteps = 1;
//		}

//		for (idxTh = 0; idxTh < nThSteps; idxTh++)
//		{
//			th = idxTh * A_ang;

//			// Compute the mean of the new Gaussian:
//			dir.x((sensorPnt.x() + R * cos(th) * cos(el)));
//			dir.y((sensorPnt.y() + R * sin(th) * cos(el)));
//			dir.z((sensorPnt.z() + R * sin(el)));

//			// If we are provided a radius for not creating modes out of it,
//			// check it:
//			bool reallyCreateIt = true;
//			if (maxDistanceFromCenter > 0)
//				reallyCreateIt =
//					dir.distanceTo(centerPoint) < maxDistanceFromCenter;

//			if (reallyCreateIt)
//			{
//				// All have equal log-weights:
//				outPDF.get(modeIdx).log_w = 0;

//				// The mean:
//				outPDF.get(modeIdx).val.mean = dir;

//				// Compute the covariance:
//				dir = dir - sensorPnt;
//				CMatrixDouble33 H = CMatrixDouble33(
//					math::generateAxisBaseFromDirection(
//						dir.x(), dir.y(),
//						dir.z()));  // 3 perpendicular & normalized vectors.

//				H.multiply_HCHt(
//					S,
//					outPDF.get(modeIdx).val.cov);  // out = H * S * ~H;
//				if (minEl == maxEl)
//				{  // We are in 2D:
//					// 3rd column/row = 0
//					CMatrixDouble33& C33 = outPDF.get(modeIdx).val.cov;
//					C33.get_unsafe(0, 2) = C33.get_unsafe(2, 0) =
//						C33.get_unsafe(1, 2) = C33.get_unsafe(2, 1) =
//							C33.get_unsafe(2, 2) = 0;
//				}

//				// Add covariance for uncertainty composition?
//				if (covarianceCompositionToAdd)
//					outPDF.get(modeIdx).val.cov += *covarianceCompositionToAdd;

//				// One more mode is used:
//				modeIdx++;

//			}  // end if reallyCreateIt

//		}  // end for idxTh
//	}  // end for idxEl

//	// resize to the number of really used modes:
//	outPDF.resize(modeIdx);

	MRPT_END
}
