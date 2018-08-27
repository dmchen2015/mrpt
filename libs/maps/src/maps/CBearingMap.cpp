/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "maps-precomp.h"  // Precomp header

#include <mrpt/maps/CBearingMap.h>
#include <mrpt/obs/CObservationBearingRange.h>
#include <mrpt/random.h>
#include <mrpt/io/CFileOutputStream.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/core/round.h>  // round()
#include <mrpt/math/geometry.h>
#include <mrpt/bayes/CParticleFilterCapable.h>
#include <mrpt/bayes/CParticleFilter.h>
#include <mrpt/math/data_utils.h>  // averageLogLikelihood()
#include <mrpt/system/os.h>
#include <mrpt/system/string_utils.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPointPDFGaussian.h>
#include <mrpt/poses/CPosePDFSOG.h>
#include <chrono>

#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/stock_objects.h>

using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::math;
using namespace mrpt::obs;
using namespace mrpt::random;
using namespace mrpt::poses;
using namespace mrpt::bayes;
using namespace mrpt::system;
using namespace mrpt::tfest;
using namespace std;

//  =========== Begin of Map definition ============
MAP_DEFINITION_REGISTER("CBearingMap,bearingMap", mrpt::maps::CBearingMap)

CBearingMap::TMapDefinition::TMapDefinition() {}
void CBearingMap::TMapDefinition::loadFromConfigFile_map_specific(
        const mrpt::config::CConfigFileBase& source,
        const std::string& sectionNamePrefix)
{
        // [<sectionNamePrefix>+"_creationOpts"]
        // const std::string sSectCreation =
        // sectionNamePrefix+string("_creationOpts");
        // MRPT_LOAD_CONFIG_VAR(resolution, float,   source,sSectCreation);

        insertionOpts.loadFromConfigFile(
                source, sectionNamePrefix + string("_insertOpts"));
        likelihoodOpts.loadFromConfigFile(
                source, sectionNamePrefix + string("_likelihoodOpts"));
}

void CBearingMap::TMapDefinition::dumpToTextStream_map_specific(
        std::ostream& out) const
{
        // LOADABLEOPTS_DUMP_VAR(resolution     , float);

        this->insertionOpts.dumpToTextStream(out);
        this->likelihoodOpts.dumpToTextStream(out);
}

mrpt::maps::CMetricMap* CBearingMap::internal_CreateFromMapDefinition(
        const mrpt::maps::TMetricMapInitializer& _def)
{
        const CBearingMap::TMapDefinition& def =
                *dynamic_cast<const CBearingMap::TMapDefinition*>(&_def);
        CBearingMap* obj = new CBearingMap();
        obj->insertionOptions = def.insertionOpts;
        obj->likelihoodOptions = def.likelihoodOpts;
        return obj;
}
//  =========== End of Map definition Block =========

IMPLEMENTS_SERIALIZABLE(CBearingMap, CMetricMap, mrpt::maps)

/*---------------------------------------------------------------
                                                Constructor
  ---------------------------------------------------------------*/
CBearingMap::CBearingMap() : m_bearings(0), likelihoodOptions(), insertionOptions()
{
}

/*---------------------------------------------------------------
                                                clear
  ---------------------------------------------------------------*/
void CBearingMap::internal_clear() { m_bearings.clear(); }
/*---------------------------------------------------------------
                                                getLandmarksCount
  ---------------------------------------------------------------*/
size_t CBearingMap::size() const { return m_bearings.size(); }
/*---------------------------------------------------------------
        Resize
  ---------------------------------------------------------------*/
void CBearingMap::resize(const size_t N) { m_bearings.resize(N); }
uint8_t CBearingMap::serializeGetVersion() const { return 1; }
void CBearingMap::serializeTo(mrpt::serialization::CArchive& out) const
{
        out << genericMapParams;  // v1

        // First, write the number of landmarks:
        const uint32_t n = m_bearings.size();
        out << n;
        // Write all landmarks:
        for (const_iterator it = begin(); it != end(); ++it) out << (*it);
}

void CBearingMap::serializeFrom(
        mrpt::serialization::CArchive& in, uint8_t version)
{
        switch (version)
        {
                case 0:
                case 1:
                {
                        if (version >= 1) in >> genericMapParams;  // v1

                        uint32_t n, i;

                        // Delete previous content of map:
                        clear();

                        // Load from stream:
                        // Read all landmarks:
                        in >> n;
                        m_bearings.resize(n);
                        for (i = 0; i < n; i++) in >> m_bearings[i];
                }
                break;
                default:
                        MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
        };
}

/*---------------------------------------------------------------
                                        computeObservationLikelihood
  ---------------------------------------------------------------*/
double CBearingMap::internal_computeObservationLikelihood(
        const CObservation* obs, const CPose3D& robotPose3D)
{
        MRPT_START

        auto t_start = std::chrono::high_resolution_clock::now();

        /* ===============================================================================================================
                Refer to the papers:
                - IROS 2008, "Efficient Probabilistic Range-Only SLAM",
                        http://www.mrpt.org/paper-ro-slam-with-sog/

                - ICRA 2008, "A Pure Probabilistic Approach to Range-Only SLAM",
                        http://www.mrpt.org/tutorials/slam-algorithms/rangeonly_slam/
           ===============================================================================================================
           */

        if (CLASS_ID(CObservationBearingRange) == obs->GetRuntimeClass())
        {
            double ret = 0.0;
            const CObservationBearingRange* o =
                static_cast<const CObservationBearingRange*>(obs);
            CBearing::Ptr bearing = nullptr;
            CPoint3D sensor3D;
            vector<CPose3D> meas_as_poses;
            o->getMeasurementAsPose3DVector(meas_as_poses, false);
            vector<CPose3D>::iterator it_poses = meas_as_poses.begin();

            for (vector<CBearingMap::TMeasBearing>::const_iterator it_obs =
                    o->sensedData.begin();
                 it_obs != o->sensedData.end(); ++it_obs, ++it_poses)
            {
                double dist = std::numeric_limits<double>::max();

                bearing = getNNBearing(*it_poses, &dist);
                //printf("bearing match: %d -> %d, distance: %lf\n", it_obs->landmarkID, bearing->m_ID, dist);

                if (bearing && !std::isnan(it_obs->range) && it_obs->range > 0)
                {
                    double sensedRange = it_obs->range;
                    double sensedYaw = it_obs->yaw;
                    switch (bearing->m_typePDF) {
                        case CBearing::pdfMonteCarlo:
                        {
                            CPose3DPDFParticles::CParticleList::const_iterator it;
                            CVectorDouble logWeights(
                                bearing->m_locationMC.m_particles.size());
                            CVectorDouble logLiks(
                                bearing->m_locationMC.m_particles.size());
                            CVectorDouble::iterator itLW, itLL;

                            for (it = bearing->m_locationMC.m_particles.begin(),
                                itLW = logWeights.begin(), itLL = logLiks.begin();
                                 it != bearing->m_locationMC.m_particles.end();
                                 ++it, ++itLW, ++itLL)
                            {
                                float expectedRange = sensor3D.distance3DTo(
                                    it->d.x, it->d.y, it->d.z);
                                // expectedRange +=
                                // float(0.1*(1-exp(-0.16*expectedRange)));

                                *itLW = it->log_w;  // Linear weight of this
                                // likelihood component
                                *itLL = -0.5 * square(
                                                   (sensedRange - expectedRange) /
                                                   likelihoodOptions.rangeStd);
                                // ret+= exp(
                                // -0.5*square((sensedRange-expectedRange)/likelihoodOptions.rangeStd)
                                // );
                            }  // end for it

                            if (logWeights.size())
                                ret += math::averageLogLikelihood(
                                    logWeights, logLiks);  // A numerically-stable
                            // method to average the
                            // likelihoods
                        }
                        break;
                        // ------------------------------
                        // PDF is Gaussian
                        // ------------------------------
                        case CBearing::pdfGauss:
                        {
                            // Compute the Jacobian H and varZ
    //                        CMatrixFixedNumeric<double, 1, 3> H;
    //                        float varZ, varR = square(likelihoodOptions.rangeStd);
    //                        float Ax =
    //                            bearing->m_locationGauss.mean.x() - sensor3D.x();
    //                        float Ay =
    //                            bearing->m_locationGauss.mean.y() - sensor3D.y();
    //                        float Az =
    //                            bearing->m_locationGauss.mean.z() - sensor3D.z();
    //                        H(0, 0) = Ax;
    //                        H(0, 1) = Ay;
    //                        H(0, 2) = Az;
    //                        float expectedRange =
    //                            sensor3D.distanceTo(beac->m_locationGauss.mean);
    //                        H *= 1.0 / expectedRange;  // sqrt(Ax*Ax+Ay*Ay+Az*Az);

    //                        varZ =
    //                            H.multiply_HCHt_scalar(beac->m_locationGauss.cov);

    //                        varZ += varR;

    //                        // Compute the mean expected range (add bias!):
    //                        // expectedRange +=
    //                        // float(0.1*(1-exp(-0.16*expectedRange)));

    //                        // Compute the likelihood:
    //                        //   lik \propto exp( -0.5* ( ^z - z  )^2 / varZ );
    //                        //   log_lik = -0.5* ( ^z - z  )^2 / varZ
    //                        ret +=
    //                            -0.5 * square(sensedRange - expectedRange) / varZ;
                        }
                        break;
                        case CBearing::pdfNO:
                        {
                            CPose3DPDFParticles::CParticleList::const_iterator it;
                            CVectorDouble logWeights(
                                bearing->m_locationMC.m_particles.size());
                            CVectorDouble logLiks(
                                bearing->m_locationMC.m_particles.size());
                            CVectorDouble::iterator itLW, itLL;
                            for (it = bearing->m_locationNoPDF.m_particles.begin(),
                                itLW = logWeights.begin(), itLL = logLiks.begin();
                                 it != bearing->m_locationNoPDF.m_particles.end();
                                 ++it, ++itLW, ++itLL)
                            {
                                float expectedRange = sensor3D.distance3DTo(
                                    it->d.x, it->d.y, it->d.z);

                                float dx = it->d.x - sensor3D.x();
                                float dy = it->d.y - sensor3D.y();

                                float expectedYaw = atan2(dy, dx);

                                // expectedRange +=
                                // float(0.1*(1-exp(-0.16*expectedRange)));

                                *itLW = it->log_w;  // Linear weight of this
                                // likelihood component
                                *itLL = -0.5 * square(
                                                   (sensedRange - expectedRange) /
                                                   likelihoodOptions.rangeStd)
                                             * square(
                                                   (sensedYaw - expectedYaw) /
                                                   likelihoodOptions.rangeYaw);
                                // ret+= exp(
                                // -0.5*square((sensedRange-expectedRange)/likelihoodOptions.rangeStd)
                                // );
                            }  // end for it

                            if (logWeights.size())
                                ret += math::averageLogLikelihood(
                                    logWeights, logLiks);  // A numerically-stable
                        }
                        default:
                            break;
                    }
                } else {
                  printf("no match found for bearing with id: %d\n",it_obs->landmarkID);
                }
            }

            return ret;
        }
        else
        {
                /********************************************************************
                                        OBSERVATION TYPE: Unknown
                ********************************************************************/
                return 0;
        }

        MRPT_END
}

/*---------------------------------------------------------------
                                                insertObservation
  ---------------------------------------------------------------*/
bool CBearingMap::internal_insertObservation(
        const mrpt::obs::CObservation* obs, const CPose3D* robotPose)
{
        MRPT_START

        CPose2D robotPose2D;
        CPose3D robotPose3D;

        if (robotPose)
        {
                robotPose2D = CPose2D(*robotPose);
                robotPose3D = (*robotPose);
        }
        else
        {
                // Default values are (0,0,0)
                return false;
        }

        if (CLASS_ID(CObservationBearingRange) == obs->GetRuntimeClass())
        {

                /********************************************************************
                                                OBSERVATION TYPE: CObservationBearingRanges
                 ********************************************************************/

                /* ===============================================================================================================
                   Refer to the papers:
                   - IROS 2008, "Efficient Probabilistic Range-Only SLAM",
                           http://www.mrpt.org/paper-ro-slam-with-sog/

                   - ICRA 2008, "A Pure Probabilistic Approach to Range-Only SLAM",
                           http://www.mrpt.org/tutorials/slam-algorithms/rangeonly_slam/
                  ===============================================================================================================
                  */

                // Here we fuse OR create the Bearing position PDF:
                // --------------------------------------------------------
            const CObservationBearingRange* o =
                static_cast<const CObservationBearingRange*>(obs);

            vector<mrpt::poses::CPose3D> meas_as_poses;
            o->getMeasurementAsPose3DVector(meas_as_poses, true);
            vector<mrpt::poses::CPose3D>::const_iterator it_map = meas_as_poses.begin();

            for (vector<CObservationBearingRange::TMeasurement>::const_iterator it =
                    o->sensedData.begin();
                 it != o->sensedData.end(); ++it, ++it_map)
            {
                CPoint3D sensorPnt(robotPose3D + o->sensorLocationOnRobot);
                double sensedRange = it->range;
                decltype(it->landmarkID) sensedID = it->landmarkID;
                CBearing::Ptr bearing = getBearingByID(sensedID);
                if (sensedRange > 0)  // Only sensible range values!
                {
                    if (!bearing)
                    {
                        // ======================================
                        //                INSERT
                        // ======================================
                        CBearing::Ptr newBearing = CBearing::Create();
                        newBearing->m_ID = sensedID;

                        if (insertionOptions.insertAsMonteCarlo)
                        {
                            // Insert as a new set of samples:
                            // ------------------------------------------------

                            newBearing->m_typePDF = CBearing::pdfMonteCarlo;

                            size_t numParts = round(
                                insertionOptions.MC_numSamplesPerMeter *
                                sensedRange);
                            ASSERT_(
                                insertionOptions.minElevation_deg <=
                                insertionOptions.maxElevation_deg);
                            double minA =
                                DEG2RAD(insertionOptions.minElevation_deg);
                            double maxA =
                                DEG2RAD(insertionOptions.maxElevation_deg);
                            newBearing->m_locationMC = CPose3DPDFParticles(numParts);
                            for (CPose3DPDFParticles::CParticleList::iterator itP =
                                     newBearing->m_locationMC.m_particles.begin();
                                 itP != newBearing->m_locationMC.m_particles.end();
                                 ++itP)
                            {
                                double th =
                                    getRandomGenerator().drawUniform(-M_PI, M_PI);
                                double el =
                                    getRandomGenerator().drawUniform(minA, maxA);
                                double R = getRandomGenerator().drawGaussian1D(
                                    sensedRange, likelihoodOptions.rangeStd);
                                itP->d.x = sensorPnt.x() + R * cos(th) * cos(el);
                                itP->d.y = sensorPnt.y() + R * sin(th) * cos(el);
                                itP->d.z = sensorPnt.z() + R * sin(el);
                            }  // end for itP
                        }
                        else if(insertionOptions.insertAsNoPDF)
                        {
                            newBearing->m_typePDF = CBearing::pdfNO;
                            //size_t numParts = round(
                                //insertionOptions.MC_numSamplesPerMeter);
                            ASSERT_(
                                insertionOptions.minElevation_deg <=
                                insertionOptions.maxElevation_deg);
//                            double minA =
//                                DEG2RAD(insertionOptions.minElevation_deg);
//                            double maxA =
//                                DEG2RAD(insertionOptions.maxElevation_deg);
                            newBearing->m_locationNoPDF = CPose3DPDFParticles(1);
                            for (CPose3DPDFParticles::CParticleList::iterator itP =
                                     newBearing->m_locationNoPDF.m_particles.begin();
                                 itP != newBearing->m_locationNoPDF.m_particles.end();
                                 ++itP)
                            {
                                MRPT_TODO("correct range insertion pdf")
                                CPose3D current_meas = *it_map;
                                itP->d.x = current_meas.x();
                                itP->d.y = current_meas.y();
                                itP->d.z = current_meas.z();
                            }  // end for itP
                        }
                        else
                        {
                            THROW_EXCEPTION("not implemented");
                            MRPT_TODO("ring sog implementation missing")
                            // Insert as a Sum of Gaussians:
                            // ------------------------------------------------
    //                        newBearing->m_typePDF = CBearing::pdfSOG;
    //                        CBearing::generateRingSOG(
    //                            sensedRange,  // Sensed range
    //                            newBearing->m_locationSOG,  // Output SOG
    //                            this,  // My CBearingMap, for options.
    //                            sensorPnt  // Sensor point
    //                        );
                        }

                        // and insert it:
                        m_bearings.push_back(newBearing);

                    }  // end insert
                    MRPT_TODO("Fusion step!");
                }
            }
    }
    else
    {
        return false;
    }

    return true;
    MRPT_END
}

/*---------------------------------------------------------------
                                determineMatching2D
  ---------------------------------------------------------------*/
void CBearingMap::determineMatching2D(
        const mrpt::maps::CMetricMap* otherMap, const CPose2D& otherMapPose,
        TMatchingPairList& correspondences, const TMatchingParams& params,
        TMatchingExtraResults& extraResults) const
{
        MRPT_UNUSED_PARAM(params);
        MRPT_START
        extraResults = TMatchingExtraResults();

        CBearingMap auxMap;
        CPose3D otherMapPose3D(otherMapPose);

        // Check the other map class:
        ASSERT_(otherMap->GetRuntimeClass() == CLASS_ID(CBearingMap));
        const CBearingMap* otherMap2 = static_cast<const CBearingMap*>(otherMap);
        vector<bool> otherCorrespondences;

        // Coordinates change:
        auxMap.changeCoordinatesReference(otherMapPose3D, otherMap2);

        // Use the 3D matching method:
        computeMatchingWith3DLandmarks(
                otherMap2, correspondences, extraResults.correspondencesRatio,
                otherCorrespondences);

        MRPT_END
}

/*---------------------------------------------------------------
                                changeCoordinatesReference
  ---------------------------------------------------------------*/
void CBearingMap::changeCoordinatesReference(const CPose3D& newOrg)
{
        // Change the reference of each individual Bearing:
        for (iterator lm = m_bearings.begin(); lm != m_bearings.end(); ++lm)
                (*lm)->changeCoordinatesReference(newOrg);
}

/*---------------------------------------------------------------
                                changeCoordinatesReference
  ---------------------------------------------------------------*/
void CBearingMap::changeCoordinatesReference(
        const CPose3D& newOrg, const mrpt::maps::CBearingMap* otherMap)
{
        // In this object we cannot apply any special speed-up: Just copy and change
        // coordinates:
        (*this) = *otherMap;
        changeCoordinatesReference(newOrg);
}

/*---------------------------------------------------------------
                                                computeMatchingWith3DLandmarks
  ---------------------------------------------------------------*/
void CBearingMap::computeMatchingWith3DLandmarks(
        const mrpt::maps::CBearingMap* anotherMap,
        TMatchingPairList& correspondences, float& correspondencesRatio,
        vector<bool>& otherCorrespondences) const
{
        MRPT_START

        TSequenceBearings::const_iterator thisIt, otherIt;
        size_t nThis, nOther;
        unsigned int j, k;
        TMatchingPair match;
        CPointPDFGaussian pointPDF_k, pointPDF_j;
        vector<bool> thisLandmarkAssigned;

        // Get the number of landmarks:
        nThis = m_bearings.size();
        nOther = anotherMap->m_bearings.size();

        // Initially no LM has a correspondence:
        thisLandmarkAssigned.resize(nThis, false);

        // Initially, set all landmarks without correspondences:
        correspondences.clear();
        otherCorrespondences.clear();
        otherCorrespondences.resize(nOther, false);
        correspondencesRatio = 0;

        for (k = 0, otherIt = anotherMap->m_bearings.begin();
                 otherIt != anotherMap->m_bearings.end(); ++otherIt, ++k)
        {
                for (j = 0, thisIt = m_bearings.begin(); thisIt != m_bearings.end();
                         ++thisIt, ++j)
                {
                        // Is it a correspondence?
                        if ((*otherIt)->m_ID == (*thisIt)->m_ID)
                        {
                                // If a previous correspondence for this LM was found, discard
                                // this one!
                                if (!thisLandmarkAssigned[j])
                                {
                                        thisLandmarkAssigned[j] = true;

                                        // OK: A correspondence found!!
                                        otherCorrespondences[k] = true;

                                        match.this_idx = j;

                                        CPose3D mean_j = m_bearings[j]->getMeanVal();

                                        match.this_x = mean_j.x();
                                        match.this_y = mean_j.y();
                                        match.this_z = mean_j.z();

                                        CPose3D mean_k = anotherMap->m_bearings[k]->getMeanVal();
                                        match.other_idx = k;
                                        match.other_x = mean_k.x();
                                        match.other_y = mean_k.y();
                                        match.other_z = mean_k.z();

                                        correspondences.push_back(match);
                                }
                        }

                }  // end of "otherIt" is SIFT

        }  // end of other it., k

        // Compute the corrs ratio:
        correspondencesRatio =
                2.0f * correspondences.size() / static_cast<float>(nThis + nOther);

        MRPT_END
}

/*---------------------------------------------------------------
                                                saveToMATLABScript3D
  ---------------------------------------------------------------*/
bool CBearingMap::saveToMATLABScript3D(
        const string& file, const char* style, float confInterval) const
{
        MRPT_UNUSED_PARAM(style);
        MRPT_UNUSED_PARAM(confInterval);

        FILE* f = os::fopen(file.c_str(), "wt");
        if (!f) return false;

        // Header:
        os::fprintf(
                f, "%%-------------------------------------------------------\n");
        os::fprintf(f, "%% File automatically generated using the MRPT method:\n");
        os::fprintf(f, "%%   'CBearingMap::saveToMATLABScript3D'\n");
        os::fprintf(f, "%%\n");
        os::fprintf(f, "%%                        ~ MRPT ~\n");
        os::fprintf(
                f, "%%  Jose Luis Blanco Claraco, University of Malaga @ 2006\n");
        os::fprintf(f, "%%  http://www.isa.uma.es/ \n");
        os::fprintf(
                f, "%%-------------------------------------------------------\n\n");

        // Main code:
        os::fprintf(f, "hold on;\n\n");
        std::vector<std::string> strs;
        string s;

        for (const_iterator it = m_bearings.begin(); it != m_bearings.end(); ++it)
        {
                (*it)->getAsMatlabDrawCommands(strs);
                mrpt::system::stringListAsString(strs, s);
                os::fprintf(f, "%s", s.c_str());
        }

        os::fprintf(f, "axis equal;grid on;");

        os::fclose(f);
        return true;
}

void CBearingMap::TLikelihoodOptions::dumpToTextStream(std::ostream& out) const
{
        out << mrpt::format(
                "\n----------- [CBearingMap::TLikelihoodOptions] ------------ \n\n");
        out << mrpt::format(
                "rangeStd                                = %f\n", rangeStd);
        out << mrpt::format("\n");
}

/*---------------------------------------------------------------
                                        loadFromConfigFile
  ---------------------------------------------------------------*/
void CBearingMap::TLikelihoodOptions::loadFromConfigFile(
        const mrpt::config::CConfigFileBase& iniFile, const string& section)
{
        rangeStd = iniFile.read_float(section.c_str(), "rangeStd", rangeStd);
}

void CBearingMap::TInsertionOptions::dumpToTextStream(std::ostream& out) const
{
        out << mrpt::format(
                "\n----------- [CBearingMap::TInsertionOptions] ------------ \n\n");

        out << mrpt::format(
                "insertAsMonteCarlo                      = %c\n",
                insertAsMonteCarlo ? 'Y' : 'N');
        out << mrpt::format(
                "minElevation_deg                        = %.03f\n", minElevation_deg);
        out << mrpt::format(
                "maxElevation_deg                        = %.03f\n", maxElevation_deg);
        out << mrpt::format(
                "MC_numSamplesPerMeter                   = %d\n",
                MC_numSamplesPerMeter);
        out << mrpt::format(
                "MC_maxStdToGauss                        = %.03f\n", MC_maxStdToGauss);
        out << mrpt::format(
                "MC_thresholdNegligible                  = %.03f\n",
                MC_thresholdNegligible);
        out << mrpt::format(
                "MC_performResampling                    = %c\n",
                MC_performResampling ? 'Y' : 'N');
        out << mrpt::format(
                "MC_afterResamplingNoise                 = %.03f\n",
                MC_afterResamplingNoise);
        out << mrpt::format(
                "SOG_thresholdNegligible                 = %.03f\n",
                SOG_thresholdNegligible);
        out << mrpt::format(
                "SOG_maxDistBetweenGaussians             = %.03f\n",
                SOG_maxDistBetweenGaussians);
        out << mrpt::format(
                "SOG_separationConstant                  = %.03f\n",
                SOG_separationConstant);

        out << mrpt::format("\n");
}

/*---------------------------------------------------------------
                                        loadFromConfigFile
  ---------------------------------------------------------------*/
void CBearingMap::TInsertionOptions::loadFromConfigFile(
        const mrpt::config::CConfigFileBase& iniFile, const string& section)
{
        MRPT_LOAD_CONFIG_VAR(insertAsMonteCarlo, bool, iniFile, section.c_str());
        MRPT_LOAD_CONFIG_VAR(maxElevation_deg, float, iniFile, section.c_str());
        MRPT_LOAD_CONFIG_VAR(minElevation_deg, float, iniFile, section.c_str());
        MRPT_LOAD_CONFIG_VAR(MC_numSamplesPerMeter, int, iniFile, section.c_str());
        MRPT_LOAD_CONFIG_VAR(MC_maxStdToGauss, float, iniFile, section.c_str());
        MRPT_LOAD_CONFIG_VAR(
                MC_thresholdNegligible, float, iniFile, section.c_str());
        MRPT_LOAD_CONFIG_VAR(MC_performResampling, bool, iniFile, section.c_str());
        MRPT_LOAD_CONFIG_VAR(
                MC_afterResamplingNoise, float, iniFile, section.c_str());
        MRPT_LOAD_CONFIG_VAR(
                SOG_thresholdNegligible, float, iniFile, section.c_str());
        MRPT_LOAD_CONFIG_VAR(
                SOG_maxDistBetweenGaussians, float, iniFile, section.c_str());
        MRPT_LOAD_CONFIG_VAR(
                SOG_separationConstant, float, iniFile, section.c_str());
}

/*---------------------------------------------------------------
                                         isEmpty
  ---------------------------------------------------------------*/
bool CBearingMap::isEmpty() const { return size() == 0; }
/*---------------------------------------------------------------
                                         simulateBearingReadings
  ---------------------------------------------------------------*/
void CBearingMap::simulateBearingReadings(
        const CPose3D& in_robotPose, const CPose3D& in_sensorLocationOnRobot,
        CObservationBearingRange& out_Observations) const
{
        TSequenceBearings::const_iterator it;
        TMeasBearing newMeas;
        CPose3D bearing3D, pose3D;
        CPointPDFGaussian bearingPDF;

        // Compute the 3D position of the sensor:
        pose3D = in_robotPose + in_sensorLocationOnRobot;

        // Clear output data:
        out_Observations.sensedData.clear();

        // For each Bearing landmark in the map:
        for (it = m_bearings.begin(); it != m_bearings.end(); ++it)
        {
                (*it)->getMean(bearing3D);

                float range = pose3D.distanceTo(bearing3D);

                if (range < out_Observations.maxSensorDistance &&
                        range > out_Observations.minSensorDistance)
                {
                        // Add noise:
                        MRPT_TODO("add noise");
//                        range += getRandomGenerator().drawGaussian1D(
//                                0, out_Observations.stdError);

                        // Fill out:
                        newMeas.landmarkID = (*it)->m_ID;
                        //newMeas.sensorLocationOnRobot = in_sensorLocationOnRobot;
                        newMeas.range = range;

                        // Insert:
                        out_Observations.sensedData.push_back(newMeas);
                }
        }  // end for it
        out_Observations.sensorLocationOnRobot = in_sensorLocationOnRobot;
        // Done!
}

/*---------------------------------------------------------------
                                         saveMetricMapRepresentationToFile
  ---------------------------------------------------------------*/
void CBearingMap::saveMetricMapRepresentationToFile(
        const string& filNamePrefix) const
{
        MRPT_START

        // Matlab:
        string fil1(filNamePrefix + string("_3D.m"));
        saveToMATLABScript3D(fil1);

        // 3D Scene:
        opengl::COpenGLScene scene;
        opengl::CSetOfObjects::Ptr obj3D =
                mrpt::make_aligned_shared<opengl::CSetOfObjects>();

        getAs3DObject(obj3D);
        auto objGround = opengl::CGridPlaneXY::Create(
                -100.0f, 100.0f, -100.0f, 100.0f, .0f, 1.f);

        scene.insert(obj3D);
        scene.insert(objGround);

        string fil2(filNamePrefix + string("_3D.3Dscene"));
        scene.saveToFile(fil2);

        // Textual representation:
        string fil3(filNamePrefix + string("_covs.txt"));
        saveToTextFile(fil3);

        // Total number of particles / modes:
        string fil4(filNamePrefix + string("_population.txt"));
        {
                FILE* f = os::fopen(fil4.c_str(), "wt");
                if (f)
                {
                        size_t nParts = 0, nGaussians = 0, nNoPDF = 0;

                        for (TSequenceBearings::const_iterator it = m_bearings.begin();
                                 it != m_bearings.end(); ++it)
                        {
                                switch ((*it)->m_typePDF)
                                {
                                        case CBearing::pdfMonteCarlo:
                                                nParts += (*it)->m_locationMC.size();
                                                break;
                                        case CBearing::pdfSOG:
                                                nGaussians += (*it)->m_locationSOG.size();
                                                break;
                                        case CBearing::pdfGauss:
                                                nGaussians++;
                                                break;
                                        case CBearing::pdfNO:
                                                nNoPDF++;
                                                break;
                                };
                        }

                        fprintf(
                                f, "%u %u", static_cast<unsigned>(nParts),
                                static_cast<unsigned>(nGaussians));
                        os::fclose(f);
                }
        }

        MRPT_END
}

/*---------------------------------------------------------------
                                                getAs3DObject
  ---------------------------------------------------------------*/
void CBearingMap::getAs3DObject(mrpt::opengl::CSetOfObjects::Ptr& outObj) const
{
        MRPT_START

        if (!genericMapParams.enableSaveAs3DObject) return;

        // ------------------------------------------------
        //  Add the XYZ corner for the current area:
        // ------------------------------------------------
        outObj->insert(opengl::stock_objects::CornerXYZ());

        // Save 3D ellipsoids or whatever representation:
        for (const_iterator it = m_bearings.begin(); it != m_bearings.end(); ++it)
                (*it)->getAs3DObject(outObj);

    for (std::vector<CBearing::Ptr>::const_iterator it = m_bearings.begin(); it != m_bearings.end(); ++it)
        (*it)->getAs3DObject(outObj);

        MRPT_END
}

/*---------------------------------------------------------------
   Computes the ratio in [0,1] of correspondences between "this" and the
 "otherMap" map, whose 6D pose relative to "this" is "otherMapPose"
 *   In the case of a multi-metric map, this returns the average between the
 maps. This method always return 0 for grid maps.
 * \param  otherMap					  [IN] The other map to compute the matching
 with.
 * \param  otherMapPose				  [IN] The 6D pose of the other map as seen
 from "this".
 * \param  maxDistForCorr			  [IN] The minimum distance between 2
 non-probabilistic map elements for counting them as a correspondence.
 * \param  maxMahaDistForCorr		  [IN] The minimum Mahalanobis distance
 between 2 probabilistic map elements for counting them as a correspondence.
 *
 * \return The matching ratio [0,1]
 * \sa computeMatchingWith2D
 ----------------------------------------------------------------*/
float CBearingMap::compute3DMatchingRatio(
        const mrpt::maps::CMetricMap* otherMap2,
        const mrpt::poses::CPose3D& otherMapPose,
        const TMatchingRatioParams& params) const
{
        MRPT_START

        // Compare to a similar map only:
        const CBearingMap* otherMap = nullptr;

        if (otherMap2->GetRuntimeClass() == CLASS_ID(CBearingMap))
                otherMap = static_cast<const CBearingMap*>(otherMap2);

        if (!otherMap) return 0;

        TMatchingPairList matchList;
        vector<bool> otherCorrespondences;
        float out_corrsRatio;

        CBearingMap modMap;

        modMap.changeCoordinatesReference(otherMapPose, otherMap);

        computeMatchingWith3DLandmarks(
                &modMap, matchList, out_corrsRatio, otherCorrespondences);

        return out_corrsRatio;

        MRPT_END
}

/*---------------------------------------------------------------
                                        getBearingByID
 ---------------------------------------------------------------*/
const CBearing::Ptr CBearingMap::getBearingByID(CBearing::TBearingID id) const
{
        for (const_iterator it = m_bearings.begin(); it != m_bearings.end(); ++it)
                if ((*it)->m_ID == id) return *it;
        return nullptr;
}


/*---------------------------------------------------------------
                                        getBearingByID
 ---------------------------------------------------------------*/

CBearing::Ptr CBearingMap::getBearingByID(CBearing::TBearingID _id)
{
    for (auto it = m_bearings.begin(); it != m_bearings.end(); ++it)
        if ((*it)->m_ID == _id) return *it;
    return nullptr;
}

CBearing::Ptr CBearingMap::getNNBearing(const mrpt::poses::CPose3D &measurement, double *dist)
{
    MRPT_TODO("check coordinate reference frame!")
    CBearing::Ptr ret = nullptr;
    double minDist = std::numeric_limits<double>::max();
    for (auto it = m_bearings.begin(); it != m_bearings.end(); ++it)
    {
        MRPT_TODO("particle position is assumed to be fixed, but in future steps also estimated")
        double distance = std::numeric_limits<double>::max();
        switch((*it)->m_typePDF)
        {
            case CBearing::pdfNO:
            case CBearing::pdfGauss:
            case CBearing::pdfMonteCarlo:
            case CBearing::pdfSOG:
            {
                CPose3D mean_pose;
                (*it)->m_locationNoPDF.getMean(mean_pose);
                MRPT_TODO("take bearing angle into account here");
                distance = measurement.distance3DTo(mean_pose.x(), mean_pose.y(), mean_pose.z());
            }
            break;
            default:
                THROW_EXCEPTION("PDF type not known");
        };

        if (distance < minDist)
        {
            minDist = distance;
            ret = *it;
        }
    }
    *dist = minDist;
    return ret;
}

/*---------------------------------------------------------------
                                        saveToTextFile
- VX VY VZ: Variances of each dimension (C11, C22, C33)
- DET2D DET3D: Determinant of the 2D and 3D covariance matrixes.
- C12, C13, C23: Cross covariances
 ---------------------------------------------------------------*/
void CBearingMap::saveToTextFile(const string& fil) const
{
        MRPT_START
        FILE* f = os::fopen(fil.c_str(), "wt");
        ASSERT_(f != nullptr);

        CPose3D p;
        CMatrixDouble66 C;

        for (const_iterator it = m_bearings.begin(); it != m_bearings.end(); ++it)
        {
                (*it)->getCovarianceAndMean(C, p);

                float D3 = C.det();
                float D2 = C(0, 0) * C(1, 1) - square(C(0, 1));
                os::fprintf(
                        f, "%i %f %f %f %e %e %e %e %e %e %e %e\n",
                        static_cast<int>((*it)->m_ID), p.x(), p.y(), p.z(), C(0, 0), C(1, 1),
                        C(2, 2), D2, D3, C(0, 1), C(1, 2), C(1, 2));
        }

        os::fclose(f);
        MRPT_END
}
