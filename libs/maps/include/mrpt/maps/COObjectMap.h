#ifndef COObjectMap_H
#define COObjectMap_H

#include <mrpt/maps/CMetricMap.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/math/CMatrix.h>
#include <mrpt/config/CLoadableOptions.h>
#include <mrpt/obs/obs_frwds.h>
#include <mrpt/maps/COObject.h>
#include <mrpt/obs/CObservationObject.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>

namespace mrpt::maps
{
/** A class for storing a map of 3D probabilistic OObjects, using a Montecarlo,
 *Gaussian, or Sum of Gaussians (SOG) representation (for range-only SLAM).
 * <br>
 *  The individual OObjects are defined as mrpt::maps::COObject objects.
 * <br>
 *  When invoking COObjectMap::insertObservation(), landmarks will be extracted
 *and fused into the map.
 *   The only currently supported observation type is
 *mrpt::obs::CObservationOObjectRanges.
 *   See insertionOptions and likelihoodOptions for parameters used when
 *creating and fusing OObject landmarks.
 * <br>
 *   Use "TInsertionOptions::insertAsMonteCarlo" to select between 2 different
 *behaviors:
 *		- Initial PDF of OObjects: MonteCarlo, after convergence, pass to
 *Gaussians; or
 *		- Initial PDF of OObjects: SOG, after convergence, a single Gaussian.
 *
 *   Refer to the papers: []
 *
  * \ingroup mrpt_maps_grp
 * \sa CMetricMap
 */
class COObjectMap : public mrpt::maps::CMetricMap
{
        DEFINE_SERIALIZABLE(COObjectMap)

   public:
        using TSequenceOObjects = std::vector<COObject::Ptr>;
        using iterator = std::vector<COObject::Ptr>::iterator;
        using const_iterator = std::vector<COObject::Ptr>::const_iterator;
        using TMeasOObject = mrpt::obs::CObservationObject::TMeasurement;

   protected:

        /** The individual OObjects */
        TSequenceOObjects m_OObjects;
        bool m_lhcEnabled = true;

        // See docs in base class
        virtual void internal_clear() override;
        virtual bool internal_insertObservation(
                const mrpt::obs::CObservation* obs,
                const mrpt::poses::CPose3D* robotPose = nullptr) override;
        double internal_computeObservationLikelihood(
                const mrpt::obs::CObservation* obs,
                const mrpt::poses::CPose3D& takenFrom) override;

   public:
        /** Constructor */
        COObjectMap();

        /** Resize the number of SOG modes */
        void resize(const size_t N);

        /** Access to individual OObjects */
        const COObject::Ptr& operator[](size_t i) const
        {
                ASSERT_(i < m_OObjects.size());
                return m_OObjects[i];
        }
        /** Access to individual OObjects */
        const COObject::Ptr& get(size_t i) const
        {
                ASSERT_(i < m_OObjects.size());
                return m_OObjects[i];
        }
        /** Access to individual OObjects */
        COObject::Ptr& operator[](size_t i)
        {
                ASSERT_(i < m_OObjects.size());
                return m_OObjects[i];
        }
        /** Access to individual OObjects */
        COObject::Ptr& get(size_t i)
        {
                ASSERT_(i < m_OObjects.size());
                return m_OObjects[i];
        }

        /** Disables likelyhood computation */
        void disable() { m_lhcEnabled = false; };

        /** Enables likelyhood computation */
        void enable() { m_lhcEnabled = true; }

        iterator begin() { return m_OObjects.begin(); }
        const_iterator begin() const { return m_OObjects.begin(); }
        iterator end() { return m_OObjects.end(); }
        const_iterator end() const { return m_OObjects.end(); }
        /** Inserts a copy of the given mode into the SOG */
        void push_back(const COObject::Ptr& m) { m_OObjects.push_back(m); }
        // See docs in base class
        float compute3DMatchingRatio(
                const mrpt::maps::CMetricMap* otherMap,
                const mrpt::poses::CPose3D& otherMapPose,
                const TMatchingRatioParams& params) const override;

        /** With this struct options are provided to the likelihood computations */
        struct TLikelihoodOptions : public mrpt::config::CLoadableOptions
        {
           public:
                void loadFromConfigFile(
                        const mrpt::config::CConfigFileBase& source,
                        const std::string& section) override;  // See base docs
                void dumpToTextStream(
                        std::ostream& out) const override;  // See base docs

                /** The standard deviation used for OObject ranges likelihood
                 * (default=0.08m).
                  */
                double rangeStd = {0.08};

                double rangeYaw = {0.25};

                bool rangeOnly = {true};

        } likelihoodOptions;

        /** This struct contains data for choosing the method by which new OObjects
         * are inserted in the map.
         */
        struct TInsertionOptions : public mrpt::config::CLoadableOptions
        {
           public:
                /** Initilization of default parameters */
                void loadFromConfigFile(
                        const mrpt::config::CConfigFileBase& source,
                        const std::string& section) override;  // See base docs
                void dumpToTextStream(
                        std::ostream& out) const override;  // See base docs

                /** Insert a new OObject as a set of montecarlo samples (default=true),
                 * or, if false, as a sum of gaussians (see mrpt::maps::COObject).
                  * \sa MC_performResampling
                  */
                bool insertAsMonteCarlo{false};

                bool insertAsNoPDF{true};

                /** Minimum and maximum elevation angles (in degrees) for inserting new
                 * OObjects at the first observation: the default values (both 0), makes
                 * the OObjects to be in the same horizontal plane that the sensors, that
                 * is, 2D SLAM - the min/max values are -90/90.
                  */
                double maxElevation_deg{0}, minElevation_deg{0};

                /** Number of particles per meter of range, i.e. per meter of the
                 * "radius of the ring".
                  */
                unsigned int MC_numSamplesPerMeter{1000};

                /** The threshold for the maximum std (X,Y,and Z) before colapsing the
                 * particles into a Gaussian PDF (default=0.4).
                  */
                float MC_maxStdToGauss = {0.4f};

                /** Threshold for the maximum difference from the maximun (log) weight
                 * in the set of samples for erasing a given sample (default=5).
                  */
                double MC_thresholdNegligible{5};

                /** If set to false (default), the samples will be generated the first
                 * time a OObject is observed, and their weights just updated
                 * subsequently - if set to "true", fewer samples will be required since
                 * the particles will be resamples when necessary, and a small "noise"
                 * will be added to avoid depletion.
                  */
                bool MC_performResampling{false};

                /** The std.dev. of the Gaussian noise to be added to each sample after
                 * resampling, only if MC_performResampling=true.
                  */
                float MC_afterResamplingNoise{0.01f};

                /** Threshold for the maximum difference from the maximun (log) weight
                 * in the SOG for erasing a given mode (default=20).
                  */
                float SOG_thresholdNegligible{20.0f};

                /** A parameter for initializing 2D/3D SOGs
                  */
                float SOG_maxDistBetweenGaussians{1.0f};

                /** Constant used to compute the std. dev. int the tangent direction
                 * when creating the Gaussians.
                  */
                float SOG_separationConstant{3.0f};
        } insertionOptions;

        /** Save to a MATLAB script which displays 3D error ellipses for the map.
         *	\param file		The name of the file to save the script to.
         *  \param style	The MATLAB-like string for the style of the lines (see
         *'help plot' in MATLAB for posibilities)
         *  \param stdCount The ellipsoids will be drawn from the center to a given
         *confidence interval in [0,1], e.g. 2 sigmas=0.95 (default is 2std = 0.95
         *confidence intervals)
         *
         *  \return Returns false if any error occured, true elsewere.
         */
        bool saveToMATLABScript3D(
                const std::string& file, const char* style = "b",
                float confInterval = 0.95f) const;

        /** Returns the stored landmarks count.
         */
        size_t size() const;

        // See docs in base class
        virtual void determineMatching2D(
                const mrpt::maps::CMetricMap* otherMap,
                const mrpt::poses::CPose2D& otherMapPose,
                mrpt::tfest::TMatchingPairList& correspondences,
                const TMatchingParams& params,
                TMatchingExtraResults& extraResults) const override;

        /** Perform a search for correspondences between "this" and another
         * lansmarks map:
          *  Firsly, the landmarks' descriptor is used to find correspondences, then
         * inconsistent ones removed by
          *    looking at their 3D poses.
          * \param otherMap [IN] The other map.
          * \param correspondences [OUT] The matched pairs between maps.
          * \param correspondencesRatio [OUT] This is NumberOfMatchings /
         * NumberOfLandmarksInTheAnotherMap
          * \param otherCorrespondences [OUT] Will be returned with a vector
         * containing "true" for the indexes of the other map's landmarks with a
         * correspondence.
          */
        void computeMatchingWith3DLandmarks(
                const mrpt::maps::COObjectMap* otherMap,
                mrpt::tfest::TMatchingPairList& correspondences,
                float& correspondencesRatio,
                std::vector<bool>& otherCorrespondences) const;

        /** Changes the reference system of the map to a given 3D pose.
          */
        void changeCoordinatesReference(const mrpt::poses::CPose3D& newOrg);

        /** Changes the reference system of the map "otherMap" and save the result
         * in "this" map.
          */
        void changeCoordinatesReference(
                const mrpt::poses::CPose3D& newOrg,
                const mrpt::maps::COObjectMap* otherMap);

        /** Returns true if the map is empty/no observation has been inserted.
           */
        bool isEmpty() const override;

        /** Simulates a reading toward each of the OObjects in the landmarks map, if
         * any.
          * \param in_robotPose This robot pose is used to simulate the ranges to
         * each OObject.
          * \param in_sensorLocationOnRobot The 3D position of the sensor on the
         * robot
          * \param out_Observations The results will be stored here. NOTICE that the
         * fields
         * "CObservationOObjectRanges::minSensorDistance","CObservationOObjectRanges::maxSensorDistance"
         * and "CObservationOObjectRanges::stdError" MUST BE FILLED OUT before
         * calling this function.
          * An observation will be generated for each OObject in the map, but notice
         * that some of them may be missed if out of the sensor maximum range.
          */
        void simulateOObjectReadings(const mrpt::poses::CPose3D& in_robotPose,
                const mrpt::poses::CPose3D &in_sensorLocationOnRobot,
                mrpt::obs::CObservationObject& out_Observations) const;

        /** This virtual method saves the map to a file "filNamePrefix"+<
          *some_file_extension >, as an image or in any other applicable way (Notice
          *that other methods to save the map may be implemented in classes
          *implementing this virtual interface).
          *  In the case of this class, these files are generated:
          *		- "filNamePrefix"+"_3D.m": A script for MATLAB for drawing landmarks
          *as
          *3D ellipses.
          *		- "filNamePrefix"+"_3D.3DScene": A 3D scene with a "ground plane
          *grid"
          *and the set of ellipsoids in 3D.
          *		- "filNamePrefix"+"_covs.m": A textual representation (see
          *saveToTextFile)
          */
        void saveMetricMapRepresentationToFile(
                const std::string& filNamePrefix) const override;

        /** Save a text file with a row per OObject, containing this 11 elements:
          *  - X Y Z: Mean values
          *  - VX VY VZ: Variances of each dimension (C11, C22, C33)
          *  - DET2D DET3D: Determinant of the 2D and 3D covariance matrixes.
          *  - C12, C13, C23: Cross covariances
          */
        void saveToTextFile(const std::string& fil) const;

        /** Returns a 3D object representing the map. */
        void getAs3DObject(mrpt::opengl::CSetOfObjects::Ptr& outObj) const override;

        /** Returns a pointer to the OObject with the given ID, or nullptr if it does
         * not exist. */
        const COObject::Ptr getOObjectByID(COObject::TOObjectID id) const;

        /** Returns a pointer to the OObject with the given ID, or nullptr if it does
         * not exist. */
        COObject::Ptr getOObjectByID(COObject::TOObjectID id);


        /**
         * @brief getNNOObject search the range OObject object via nearest neighbor search
         * @param measurement
         * @return
         */
        COObject::Ptr getNNOObject(const mrpt::poses::CPose3D &measurement, double *dist);


        MAP_DEFINITION_START(COObjectMap)
        /** Observations insertion options */
        mrpt::maps::COObjectMap::TInsertionOptions insertionOpts;
        /** Probabilistic observation likelihood options */
        mrpt::maps::COObjectMap::TLikelihoodOptions likelihoodOpts;
        MAP_DEFINITION_END(COObjectMap)

};  // End of class def.
}

#endif
