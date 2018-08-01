#ifndef CBearingMap_H
#define CBearingMap_H

#include <mrpt/maps/CMetricMap.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/math/CMatrix.h>
#include <mrpt/config/CLoadableOptions.h>
#include <mrpt/obs/obs_frwds.h>
#include <mrpt/obs/CObservationBearingRange.h>
#include <mrpt/maps/CBearing.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>

namespace mrpt::maps
{
/** A class for storing a map of 3D probabilistic Bearings, using a Montecarlo,
 *Gaussian, or Sum of Gaussians (SOG) representation (for range-only SLAM).
 * <br>
 *  The individual Bearings are defined as mrpt::maps::CBearing objects.
 * <br>
 *  When invoking CBearingMap::insertObservation(), landmarks will be extracted
 *and fused into the map.
 *   The only currently supported observation type is
 *mrpt::obs::CObservationBearingRanges.
 *   See insertionOptions and likelihoodOptions for parameters used when
 *creating and fusing Bearing landmarks.
 * <br>
 *   Use "TInsertionOptions::insertAsMonteCarlo" to select between 2 different
 *behaviors:
 *		- Initial PDF of Bearings: MonteCarlo, after convergence, pass to
 *Gaussians; or
 *		- Initial PDF of Bearings: SOG, after convergence, a single Gaussian.
 *
 *   Refer to the papers: []
 *
  * \ingroup mrpt_maps_grp
 * \sa CMetricMap
 */
class CBearingMap : public mrpt::maps::CMetricMap
{
        DEFINE_SERIALIZABLE(CBearingMap)

   public:
        using TSequenceBearings = std::vector<CBearing::Ptr>;
        using iterator = std::vector<CBearing::Ptr>::iterator;
        using const_iterator = std::vector<CBearing::Ptr>::const_iterator;
        using TMeasBearing = mrpt::obs::CObservationBearingRange::TMeasurement;

   protected:

        /** The individual bearings */
        TSequenceBearings m_bearings;

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
        CBearingMap();

        /** Resize the number of SOG modes */
        void resize(const size_t N);

        /** Access to individual Bearings */
        const CBearing::Ptr& operator[](size_t i) const
        {
                ASSERT_(i < m_bearings.size());
                return m_bearings[i];
        }
        /** Access to individual Bearings */
        const CBearing::Ptr& get(size_t i) const
        {
                ASSERT_(i < m_bearings.size());
                return m_bearings[i];
        }
        /** Access to individual Bearings */
        CBearing::Ptr& operator[](size_t i)
        {
                ASSERT_(i < m_bearings.size());
                return m_bearings[i];
        }
        /** Access to individual Bearings */
        CBearing::Ptr& get(size_t i)
        {
                ASSERT_(i < m_bearings.size());
                return m_bearings[i];
        }

        iterator begin() { return m_bearings.begin(); }
        const_iterator begin() const { return m_bearings.begin(); }
        iterator end() { return m_bearings.end(); }
        const_iterator end() const { return m_bearings.end(); }
        /** Inserts a copy of the given mode into the SOG */
        void push_back(const CBearing::Ptr& m) { m_bearings.push_back(m); }
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

                /** The standard deviation used for Bearing ranges likelihood
                 * (default=0.08m).
                  */
                double rangeStd = {0.08};
        } likelihoodOptions;

        /** This struct contains data for choosing the method by which new Bearings
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

                /** Insert a new Bearing as a set of montecarlo samples (default=true),
                 * or, if false, as a sum of gaussians (see mrpt::maps::CBearing).
                  * \sa MC_performResampling
                  */
                bool insertAsMonteCarlo{false};

                bool insertAsNoPDF{true};

                /** Minimum and maximum elevation angles (in degrees) for inserting new
                 * Bearings at the first observation: the default values (both 0), makes
                 * the Bearings to be in the same horizontal plane that the sensors, that
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
                 * time a Bearing is observed, and their weights just updated
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
                const mrpt::maps::CBearingMap* otherMap,
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
                const mrpt::maps::CBearingMap* otherMap);

        /** Returns true if the map is empty/no observation has been inserted.
           */
        bool isEmpty() const override;

        /** Simulates a reading toward each of the Bearings in the landmarks map, if
         * any.
          * \param in_robotPose This robot pose is used to simulate the ranges to
         * each Bearing.
          * \param in_sensorLocationOnRobot The 3D position of the sensor on the
         * robot
          * \param out_Observations The results will be stored here. NOTICE that the
         * fields
         * "CObservationBearingRanges::minSensorDistance","CObservationBearingRanges::maxSensorDistance"
         * and "CObservationBearingRanges::stdError" MUST BE FILLED OUT before
         * calling this function.
          * An observation will be generated for each Bearing in the map, but notice
         * that some of them may be missed if out of the sensor maximum range.
          */
        void simulateBearingReadings(const mrpt::poses::CPose3D& in_robotPose,
                const mrpt::poses::CPose3D &in_sensorLocationOnRobot,
                mrpt::obs::CObservationBearingRange& out_Observations) const;

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

        /** Save a text file with a row per Bearing, containing this 11 elements:
          *  - X Y Z: Mean values
          *  - VX VY VZ: Variances of each dimension (C11, C22, C33)
          *  - DET2D DET3D: Determinant of the 2D and 3D covariance matrixes.
          *  - C12, C13, C23: Cross covariances
          */
        void saveToTextFile(const std::string& fil) const;

        /** Returns a 3D object representing the map. */
        void getAs3DObject(mrpt::opengl::CSetOfObjects::Ptr& outObj) const override;

        /** Returns a pointer to the Bearing with the given ID, or nullptr if it does
         * not exist. */
        const CBearing::Ptr getBearingByID(CBearing::TBearingID id) const;

        /** Returns a pointer to the Bearing with the given ID, or nullptr if it does
         * not exist. */
        CBearing::Ptr getBearingByID(CBearing::TBearingID id);


        /**
         * @brief getNNBearing search the range bearing object via nearest neighbor search
         * @param measurement
         * @return
         */
        CBearing::Ptr getNNBearing(const mrpt::poses::CPose3D &measurement, double *dist);


        MAP_DEFINITION_START(CBearingMap)
        /** Observations insertion options */
        mrpt::maps::CBearingMap::TInsertionOptions insertionOpts;
        /** Probabilistic observation likelihood options */
        mrpt::maps::CBearingMap::TLikelihoodOptions likelihoodOpts;
        MAP_DEFINITION_END(CBearingMap)

};  // End of class def.
}

#endif
