/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "maps-precomp.h"  // Precomp header

#include <mrpt/maps/COObjectMap.h>
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
#include <mrpt/system/COutputLogger.h>

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
MAP_DEFINITION_REGISTER("COObjectMap,OObjectMap", mrpt::maps::COObjectMap)

COObjectMap::TMapDefinition::TMapDefinition() {}
void COObjectMap::TMapDefinition::loadFromConfigFile_map_specific(
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

void COObjectMap::TMapDefinition::dumpToTextStream_map_specific(
        std::ostream& out) const
{
        // LOADABLEOPTS_DUMP_VAR(resolution     , float);

        this->insertionOpts.dumpToTextStream(out);
        this->likelihoodOpts.dumpToTextStream(out);
}

mrpt::maps::CMetricMap* COObjectMap::internal_CreateFromMapDefinition(
        const mrpt::maps::TMetricMapInitializer& _def)
{
        const COObjectMap::TMapDefinition& def =
                *dynamic_cast<const COObjectMap::TMapDefinition*>(&_def);
        COObjectMap* obj = new COObjectMap();
        obj->insertionOptions = def.insertionOpts;
        obj->likelihoodOptions = def.likelihoodOpts;
        return obj;
}
//  =========== End of Map definition Block =========

IMPLEMENTS_SERIALIZABLE(COObjectMap, CMetricMap, mrpt::maps)

/*---------------------------------------------------------------
                                                Constructor
  ---------------------------------------------------------------*/
COObjectMap::COObjectMap() : m_OObjects(0), likelihoodOptions(), insertionOptions()
{
}

/*---------------------------------------------------------------
                                                clear
  ---------------------------------------------------------------*/
void COObjectMap::internal_clear() { m_OObjects.clear(); }
/*---------------------------------------------------------------
                                                getLandmarksCount
  ---------------------------------------------------------------*/
size_t COObjectMap::size() const { return m_OObjects.size(); }
/*---------------------------------------------------------------
        Resize
  ---------------------------------------------------------------*/
void COObjectMap::resize(const size_t N) { m_OObjects.resize(N); }
uint8_t COObjectMap::serializeGetVersion() const { return 1; }
void COObjectMap::serializeTo(mrpt::serialization::CArchive& out) const
{
        out << genericMapParams;  // v1

        // First, write the number of landmarks:
        const uint32_t n = m_OObjects.size();
        out << n;
        // Write all landmarks:
        for (const_iterator it = begin(); it != end(); ++it) out << (*it);
}

void COObjectMap::serializeFrom(
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
                        m_OObjects.resize(n);
                        for (i = 0; i < n; i++) in >> m_OObjects[i];
                }
                break;
                default:
                        MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
        };
}

/*---------------------------------------------------------------
                                        computeObservationLikelihood
  ---------------------------------------------------------------*/
double COObjectMap::internal_computeObservationLikelihood(
        const CObservation* obs, const CPose3D& robotPose3D)
{
        if (!m_lhcEnabled)
          return 0.0;

        MRPT_START

        /* ===============================================================================================================
                Refer to the papers:
                - IROS 2008, "Efficient Probabilistic Range-Only SLAM",
                        http://www.mrpt.org/paper-ro-slam-with-sog/

                - ICRA 2008, "A Pure Probabilistic Approach to Range-Only SLAM",
                        http://www.mrpt.org/tutorials/slam-algorithms/rangeonly_slam/
           ===============================================================================================================
           */
                
        if (CLASS_ID(CObservationObject) == obs->GetRuntimeClass())
        {
            double ret = 0.0;
            const CObservationObject* o =
                static_cast<const CObservationObject*>(obs);
            CPose3D sensorPose3D = robotPose3D + o->sensorLocationOnRobot;

            vector<COObjectMap::TMeasOObject>::const_iterator it_obs;

            for (it_obs = o->sensedData.begin();
                 it_obs != o->sensedData.end(); ++it_obs)
            {
                //OObject = getNNOObject(*it_poses, &dist);
                //COObject::Ptr oObjectRef = getOObjectByID(it_obs->landmarkID);
								double d_smallest;
								const COObject::Ptr oObjectRef = getOObjectByNN(sensorPose3D + it_obs->pose_so, &d_smallest);

                if (oObjectRef)// && !std::isnan(it_obs->range) && it_obs->range > 0)
                {
                    CPose3D pose_so = it_obs->pose_so;
                    CPose3D pose_wo = sensorPose3D + it_obs->pose_so;
                    const float sensedRange = sensorPose3D.distance3DTo(pose_wo.x(),
                                                                        pose_wo.y(),
                                                                        pose_wo.z());
                    
                    const float sensedYPR[3] = {static_cast<float>(pose_wo.yaw()), 
                                                static_cast<float>(pose_wo.pitch()), 
                                                static_cast<float>(pose_wo.roll())};

                    switch (oObjectRef->m_typePDF) {
                        case COObject::pdfMonteCarlo:
                        {
                            CPose3DPDFParticles::CParticleList::const_iterator it;
                            CVectorDouble logWeights(
                                oObjectRef->m_locationMC.m_particles.size());
                            CVectorDouble logLiks(
                                oObjectRef->m_locationMC.m_particles.size());
                            CVectorDouble::iterator itLW, itLL;

                            for (it = oObjectRef->m_locationMC.m_particles.begin(),
                                itLW = logWeights.begin(), itLL = logLiks.begin();
                                 it != oObjectRef->m_locationMC.m_particles.end();
                                 ++it, ++itLW, ++itLL)
                            {
                            }  // end for it
                        }
                        break;
                        // ------------------------------
                        // PDF is Gaussian
                        // ------------------------------
                        case COObject::pdfGauss:
                        {
                        }
                        break;
                        case COObject::pdfNO:
                        {
                            CPose3DPDFParticles::CParticleList::const_iterator it;
                            CVectorDouble logWeights(
                                oObjectRef->m_locationNoPDF.m_particles.size());
                            CVectorDouble logLiks(
                                oObjectRef->m_locationNoPDF.m_particles.size());
                            CVectorDouble::iterator itLW, itLL;
                            for (it = oObjectRef->m_locationNoPDF.m_particles.begin(),
                                itLW = logWeights.begin(), itLL = logLiks.begin();
                                 it != oObjectRef->m_locationNoPDF.m_particles.end();
                                 ++it, ++itLW, ++itLL)
                            {
                                //it has ground truth stored
                                float expectedRange = sensorPose3D.distance3DTo(
                                    it->d.x, it->d.y, it->d.z);
                                float expectedYPR[3] = { static_cast<float>(it->d.yaw), 
                                                         static_cast<float>(it->d.pitch),
                                                         static_cast<float>(it->d.roll) 
                                                       };

                                //float dx = it->d.x - sensorPose3D.x();
                                //float dy = it->d.y - sensorPose3D.y();

                                //float expectedYaw = atan2(dy, dx);

                                //*itLW = 0.0;  // Linear weight of this
                                
                                if (likelihoodOptions.rangeOnly)
                                {
                                    *itLL = -0.5 * square((sensedRange - expectedRange) /
                                                       likelihoodOptions.rangeStd);
                                } 
                                else 
                                {
                                    CVectorDouble vd(4);
																		double weightRange = square(expectedRange);
                                    vd[3] = square(sensedRange-expectedRange / likelihoodOptions.rangeStd);
                                    
                                    for (size_t a_idx=0; a_idx < 3; ++a_idx)
                                    {
                                    	vd[a_idx] = square(
																													atan2(sin(sensedYPR[a_idx]-expectedYPR[a_idx]), 
                                                           		 	cos(sensedYPR[a_idx]-expectedYPR[a_idx])) 
																													/ likelihoodOptions.rangeYaw
                                                    		 );
                                    }

																		float angleSum = vd[0] + vd[1] + vd[2];

																		if (fabs(angleSum) > 0.0)
																		{
																			*itLL = -0.5 * weightRange * vd[3] * angleSum;
																		}
																		else 
																		{
                                    	*itLL = -0.5 * weightRange * vd[3];
																		}
                                }
                            }  // end for it

                            if (logWeights.size())
                                ret += math::averageLogLikelihood(logLiks);//math::averageLogLikelihood(
                                    //logWeights, logLiks);  // A numerically-stable
                        }
                        default:
                          break;
                    }
                } 
								else 
								{
                  printf("no match found for OObject with id: %d\n",it_obs->landmarkID);
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
bool COObjectMap::internal_insertObservation(
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
                printf("ROBOT POSE NOT SET\n");
                robotPose2D = CPose2D(0,0,0);
                robotPose3D = CPose3D(0,0,0,0,0);
        }

        if (CLASS_ID(CObservationObject) == obs->GetRuntimeClass())
        {
                /********************************************************************
                                                OBSERVATION TYPE: CObservationObjects
                 ********************************************************************/

                /* ===============================================================================================================
                   Refer to the papers:
                   - IROS 2008, "Efficient Probabilistic Range-Only SLAM",
                           http://www.mrpt.org/paper-ro-slam-with-sog/

                   - ICRA 2008, "A Pure Probabilistic Approach to Range-Only SLAM",
                           http://www.mrpt.org/tutorials/slam-algorithms/rangeonly_slam/
                  ===============================================================================================================
                  */

                // Here we fuse OR create the OObject position PDF:
                // --------------------------------------------------------
            const CObservationObject* o =
                static_cast<const CObservationObject*>(obs);

            for (vector<CObservationObject::TMeasurement>::const_iterator it =
                    o->sensedData.begin();
                 it != o->sensedData.end(); ++it)
            {
                CPose3D sensorPose = robotPose3D + o->sensorLocationOnRobot;
                double sensedRange = it->range;
                decltype(it->landmarkID) sensedID = it->landmarkID;
                COObject::Ptr OObject = getOObjectByID(sensedID);
                if (sensedRange > 0)  // Only sensible range values!
                {
                    if (!OObject)
                    {
                        // ======================================
                        //                INSERT
                        // ======================================
                        COObject::Ptr newOObject = COObject::Create();
                        newOObject->m_ID = sensedID;

                        if (insertionOptions.insertAsMonteCarlo)
                        {
                            // Insert as a new set of samples:
                            // ------------------------------------------------

                            newOObject->m_typePDF = COObject::pdfMonteCarlo;

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
                            newOObject->m_locationMC = CPose3DPDFParticles(numParts);
                            for (CPose3DPDFParticles::CParticleList::iterator itP =
                                     newOObject->m_locationMC.m_particles.begin();
                                 itP != newOObject->m_locationMC.m_particles.end();
                                 ++itP)
                            {
                                double th =
                                    getRandomGenerator().drawUniform(-M_PI, M_PI);
                                double el =
                                    getRandomGenerator().drawUniform(minA, maxA);
                                double R = getRandomGenerator().drawGaussian1D(
                                    sensedRange, likelihoodOptions.rangeStd);
                                itP->d.x = sensorPose.x() + R * cos(th) * cos(el);
                                itP->d.y = sensorPose.y() + R * sin(th) * cos(el);
                                itP->d.z = sensorPose.z() + R * sin(el);
                            }  // end for itP
                        }
                        else if(insertionOptions.insertAsNoPDF)
                        {
                            newOObject->m_typePDF = COObject::pdfNO;
                            //size_t numParts = round(
                                //insertionOptions.MC_numSamplesPerMeter);
                            ASSERT_(
                                insertionOptions.minElevation_deg <=
                                insertionOptions.maxElevation_deg);
//                            double minA =
//                                DEG2RAD(insertionOptions.minElevation_deg);
//                            double maxA =
//                                DEG2RAD(insertionOptions.maxElevation_deg);
                            newOObject->m_locationNoPDF = CPose3DPDFParticles(1);
                            CPose3DPDFParticles::CParticleList::iterator itP;
                            unsigned int iii;
                            for (itP = newOObject->m_locationNoPDF.m_particles.begin(),
                                 iii = 0;
                                 itP != newOObject->m_locationNoPDF.m_particles.end();
                                 ++itP, ++iii)
                            {

                                //itP->d = (sensorPnt + tmp_p).asTPose();
                                //always in world space coordinate system
                                //const double c_yaw = cos(it->yaw);
                                //const double s_yaw = sin(it->yaw);
                              	CPose3D pose_wo;
                                const double max_double = std::numeric_limits<double>::max();
                              	//if ( it->pose_wo != CPose3D(max_double,max_double,max_double,0,0,0) )
                               // {
                               //   pose_wo = it->pose_wo;
                               // } 
                                if ( it->pose_so != CPose3D(max_double,max_double,max_double,0,0,0) )
                                {
                                  pose_wo = sensorPose + it->pose_so;
                                }
                                else 
                                {
                                  MRPT_TODO("use angular information and shape vars");
                                }
                                
                                MRPT_TODO("dont forget to set z when needed");
                                MRPT_TODO("angular info update...");
                                itP->d = pose_wo.asTPose();
                                itP->d.z = 0;
                                itP->log_w = 1.0;
                                  
                                //itP->d.x = OObject_ws.x();
                                //itP->d.y = OObject_ws.y();
                                //itP->d.z = 0.0;
                                //itP->d.yaw = OObject_ws.yaw();
                                //itP->d.pitch = OObject_ws.pitch();
                                //itP->d.roll = OObject_ws.roll();

                                //itP->d.z = sensorPnt.z() + sensedRange * sin(el);
                            }  // end for itP
                        }
                        else
                        {
                            THROW_EXCEPTION("not implemented");
                            MRPT_TODO("ring sog implementation missing")
                            // Insert as a Sum of Gaussians:
                            // ------------------------------------------------
    //                        newOObject->m_typePDF = COObject::pdfSOG;
    //                        COObject::generateRingSOG(
    //                            sensedRange,  // Sensed range
    //                            newOObject->m_locationSOG,  // Output SOG
    //                            this,  // My COObjectMap, for options.
    //                            sensorPnt  // Sensor point
    //                        );
                        }

                        // and insert it:
                        m_OObjects.push_back(newOObject);

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
void COObjectMap::determineMatching2D(
        const mrpt::maps::CMetricMap* otherMap, const CPose2D& otherMapPose,
        TMatchingPairList& correspondences, const TMatchingParams& params,
        TMatchingExtraResults& extraResults) const
{
        MRPT_UNUSED_PARAM(params);
        MRPT_START
        extraResults = TMatchingExtraResults();

        COObjectMap auxMap;
        CPose3D otherMapPose3D(otherMapPose);

        // Check the other map class:
        ASSERT_(otherMap->GetRuntimeClass() == CLASS_ID(COObjectMap));
        const COObjectMap* otherMap2 = static_cast<const COObjectMap*>(otherMap);
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
void COObjectMap::changeCoordinatesReference(const CPose3D& newOrg)
{
        // Change the reference of each individual OObject:
        for (iterator lm = m_OObjects.begin(); lm != m_OObjects.end(); ++lm)
                (*lm)->changeCoordinatesReference(newOrg);
}

/*---------------------------------------------------------------
                                changeCoordinatesReference
  ---------------------------------------------------------------*/
void COObjectMap::changeCoordinatesReference(
        const CPose3D& newOrg, const mrpt::maps::COObjectMap* otherMap)
{
        // In this object we cannot apply any special speed-up: Just copy and change
        // coordinates:
        (*this) = *otherMap;
        changeCoordinatesReference(newOrg);
}

/*---------------------------------------------------------------
                                                computeMatchingWith3DLandmarks
  ---------------------------------------------------------------*/
void COObjectMap::computeMatchingWith3DLandmarks(
        const mrpt::maps::COObjectMap* anotherMap,
        TMatchingPairList& correspondences, float& correspondencesRatio,
        vector<bool>& otherCorrespondences) const
{
        MRPT_START

        TSequenceOObjects::const_iterator thisIt, otherIt;
        size_t nThis, nOther;
        unsigned int j, k;
        TMatchingPair match;
        CPointPDFGaussian pointPDF_k, pointPDF_j;
        vector<bool> thisLandmarkAssigned;

        // Get the number of landmarks:
        nThis = m_OObjects.size();
        nOther = anotherMap->m_OObjects.size();

        // Initially no LM has a correspondence:
        thisLandmarkAssigned.resize(nThis, false);

        // Initially, set all landmarks without correspondences:
        correspondences.clear();
        otherCorrespondences.clear();
        otherCorrespondences.resize(nOther, false);
        correspondencesRatio = 0;

        for (k = 0, otherIt = anotherMap->m_OObjects.begin();
                 otherIt != anotherMap->m_OObjects.end(); ++otherIt, ++k)
        {
                for (j = 0, thisIt = m_OObjects.begin(); thisIt != m_OObjects.end();
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

                                        CPose3D mean_j = m_OObjects[j]->getMeanVal();

                                        match.this_x = mean_j.x();
                                        match.this_y = mean_j.y();
                                        match.this_z = mean_j.z();

                                        CPose3D mean_k = anotherMap->m_OObjects[k]->getMeanVal();
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
bool COObjectMap::saveToMATLABScript3D(
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
        os::fprintf(f, "%%   'COObjectMap::saveToMATLABScript3D'\n");
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

        for (const_iterator it = m_OObjects.begin(); it != m_OObjects.end(); ++it)
        {
                (*it)->getAsMatlabDrawCommands(strs);
                mrpt::system::stringListAsString(strs, s);
                os::fprintf(f, "%s", s.c_str());
        }

        os::fprintf(f, "axis equal;grid on;");

        os::fclose(f);
        return true;
}

void COObjectMap::TLikelihoodOptions::dumpToTextStream(std::ostream& out) const
{
        out << mrpt::format(
                "\n----------- [COObjectMap::TLikelihoodOptions] ------------ \n\n");
        out << mrpt::format(
                "rangeStd                                = %f\n", rangeStd);
        out << mrpt::format("\n");
}

/*---------------------------------------------------------------
                                        loadFromConfigFile
  ---------------------------------------------------------------*/
void COObjectMap::TLikelihoodOptions::loadFromConfigFile(
        const mrpt::config::CConfigFileBase& iniFile, const string& section)
{
        rangeStd = iniFile.read_float(section.c_str(), "rangeStd", rangeStd);
        rangeYaw = iniFile.read_float(section.c_str(), "rangeYaw", rangeYaw);
        rangeOnly = iniFile.read_bool(section.c_str(), "rangeOnly", rangeOnly);
}

void COObjectMap::TInsertionOptions::dumpToTextStream(std::ostream& out) const
{
        out << mrpt::format(
                "\n----------- [COObjectMap::TInsertionOptions] ------------ \n\n");

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
void COObjectMap::TInsertionOptions::loadFromConfigFile(
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
bool COObjectMap::isEmpty() const { return size() == 0; }
/*---------------------------------------------------------------
                                         simulateOObjectReadings
  ---------------------------------------------------------------*/
void COObjectMap::simulateOObjectReadings(
        const CPose3D& in_robotPose, const CPose3D& in_sensorLocationOnRobot,
        CObservationObject& out_Observations) const
{
        TSequenceOObjects::const_iterator it;
        TMeasOObject newMeas;
        CPose3D OObject3D, pose3D;
        CPointPDFGaussian OObjectPDF;

        // Compute the 3D position of the sensor:
        pose3D = in_robotPose + in_sensorLocationOnRobot;

        // Clear output data:
        out_Observations.sensedData.clear();

        // For each OObject landmark in the map:
        for (it = m_OObjects.begin(); it != m_OObjects.end(); ++it)
        {
                (*it)->getMean(OObject3D);

                float range = pose3D.distanceTo(OObject3D);

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
void COObjectMap::saveMetricMapRepresentationToFile(
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

                        for (TSequenceOObjects::const_iterator it = m_OObjects.begin();
                                 it != m_OObjects.end(); ++it)
                        {
                                switch ((*it)->m_typePDF)
                                {
                                        case COObject::pdfMonteCarlo:
                                                nParts += (*it)->m_locationMC.size();
                                                break;
                                        case COObject::pdfSOG:
                                                nGaussians += (*it)->m_locationSOG.size();
                                                break;
                                        case COObject::pdfGauss:
                                                nGaussians++;
                                                break;
                                        case COObject::pdfNO:
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
void COObjectMap::getAs3DObject(mrpt::opengl::CSetOfObjects::Ptr& outObj) const
{
        MRPT_START

        if (!genericMapParams.enableSaveAs3DObject) return;

        // ------------------------------------------------
        //  Add the XYZ corner for the current area:
        // ------------------------------------------------
        outObj->insert(opengl::stock_objects::CornerXYZ());

        // Save 3D ellipsoids or whatever representation:
        for (const_iterator it = m_OObjects.begin(); it != m_OObjects.end(); ++it)
                (*it)->getAs3DObject(outObj);

        for (std::vector<COObject::Ptr>::const_iterator it = m_OObjects.begin(); it != m_OObjects.end(); ++it)
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
float COObjectMap::compute3DMatchingRatio(
        const mrpt::maps::CMetricMap* otherMap2,
        const mrpt::poses::CPose3D& otherMapPose,
        const TMatchingRatioParams& params) const
{
        MRPT_START

        // Compare to a similar map only:
        const COObjectMap* otherMap = nullptr;

        if (otherMap2->GetRuntimeClass() == CLASS_ID(COObjectMap))
                otherMap = static_cast<const COObjectMap*>(otherMap2);

        if (!otherMap) return 0;

        TMatchingPairList matchList;
        vector<bool> otherCorrespondences;
        float out_corrsRatio;

        COObjectMap modMap;

        modMap.changeCoordinatesReference(otherMapPose, otherMap);

        computeMatchingWith3DLandmarks(
                &modMap, matchList, out_corrsRatio, otherCorrespondences);

        return out_corrsRatio;

        MRPT_END
}

/*---------------------------------------------------------------
                                        getOObjectByID
 ---------------------------------------------------------------*/
const COObject::Ptr COObjectMap::getOObjectByID(COObject::TOObjectID id) const
{
        for (const_iterator it = m_OObjects.begin(); it != m_OObjects.end(); ++it)
                if ((*it)->m_ID == id) return *it;
        return nullptr;
}


/*---------------------------------------------------------------
                                        getOObjectByID
 ---------------------------------------------------------------*/

COObject::Ptr COObjectMap::getOObjectByID(COObject::TOObjectID _id)
{
    for (auto it = m_OObjects.begin(); it != m_OObjects.end(); ++it)
        if ((*it)->m_ID == _id) return *it;
    return nullptr;
}

const COObject::Ptr COObjectMap::getOObjectByNN(const mrpt::poses::CPose3D &measurement, double *dist)
{
    MRPT_TODO("check coordinate reference frame!")
    COObject::Ptr ret;
    double minDist = std::numeric_limits<double>::max();

    for (auto it = m_OObjects.begin(); it != m_OObjects.end(); ++it)
    {
        MRPT_TODO("particle position is assumed to be fixed, but in future steps also estimated")
				const auto refOObject = *it;

        CPose3D mean_pose;
        refOObject->m_locationNoPDF.getMean(mean_pose);
        const double distance = measurement.distanceTo(mean_pose);

        if (distance < minDist)
        {
            minDist = distance;
            ret = *it;
        }
    }

		if (dist)
		{
    	*dist = minDist;
		}

    return ret;
}

const COObject::Ptr COObjectMap::getOObjectByNN(COObject::Ptr objectObs, double *dist)
{
		CPose3D pose_wo;
		objectObs->m_locationNoPDF.getMean(pose_wo);
		return getOObjectByNN(pose_wo, dist);
}

/*---------------------------------------------------------------
                                        saveToTextFile
- VX VY VZ: Variances of each dimension (C11, C22, C33)
- DET2D DET3D: Determinant of the 2D and 3D covariance matrixes.
- C12, C13, C23: Cross covariances
 ---------------------------------------------------------------*/
void COObjectMap::saveToTextFile(const string& fil) const
{
        MRPT_START
        FILE* f = os::fopen(fil.c_str(), "wt");
        ASSERT_(f != nullptr);

        CPose3D p;
        CMatrixDouble66 C;

        for (const_iterator it = m_OObjects.begin(); it != m_OObjects.end(); ++it)
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
