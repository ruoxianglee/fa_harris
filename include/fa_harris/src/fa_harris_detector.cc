/****************************************************************************************
*  FA-Harris corner detection. 
*
*  This method will save corner events to `/corner/scene_/fa_harris.txt` 
*  with (x, y, t, p). Modify `corner.launch` file to choose scene before `launch`.
*
*  Author: Ruoxiang Li
*  Date: 2019.1.20
*
****************************************************************************************/

#include "fa_harris_detector.h"
#include <vector>
#include <algorithm>

using namespace std;

namespace fa_harris  {

FAHarrisDetector::FAHarrisDetector() :
    kSmallCircle_{{0, 3}, {1, 3}, {2, 2}, {3, 1},
              {3, 0}, {3, -1}, {2, -2}, {1, -3},
              {0, -3}, {-1, -3}, {-2, -2}, {-3, -1},
              {-3, 0}, {-3, 1}, {-2, 2}, {-1, 3}},
    kLargeCircle_{{0, 4}, {1, 4}, {2, 3}, {3, 2},
              {4, 1}, {4, 0}, {4, -1}, {3, -2},
              {2, -3}, {1, -4}, {0, -4}, {-1, -4},
              {-2, -3}, {-3, -2}, {-4, -1}, {-4, 0},
              {-4, 1}, {-3, 2}, {-2, 3}, {-1, 4}}
{
    // parameters
    patch_ = Eigen::MatrixXi::Constant(9, 9, 0);
    latest_event_local_ = 25;
    window_size_ = 4;         
    kernel_size_ = 5;        
    harris_threshold_ = 8.0;

    Eigen::VectorXd Dx = Eigen::VectorXd(kernel_size_);                 
    Eigen::VectorXd Sx = Eigen::VectorXd(kernel_size_);
    for (int i = 0; i < kernel_size_; i++)
    {                                 
        Sx[i] = factorial(kernel_size_ - 1)/
                (factorial(kernel_size_ - 1 - i) * factorial(i));
        Dx[i] = pasc(i, kernel_size_-2) - pasc(i - 1, kernel_size_ - 2);
    }                                                                                                                             
    Gx_ = Sx * Dx.transpose();
    Gx_ = Gx_ / Gx_.maxCoeff();

    const double sigma = 1.;
    const double A = 1. / (2. * M_PI * sigma*sigma);
    const int l2 = (2 * window_size_ + 2 - kernel_size_) / 2;
    h_ = Eigen::MatrixXd(2 * l2 + 1, 2 * l2 + 1);
    for (int x = -l2; x <= l2; x++)
    {
        for (int y = -l2; y <= l2; y++)
        {
            const double h_xy = A * exp(-(x * x + y * y) / (2 * sigma * sigma));
            h_(l2 + x, l2 + y) = h_xy;
        }
    }
    h_ /= h_.sum();

    // Initialize Surface of Active Events to 0-timestamp
    sae_[0] = Eigen::MatrixXd::Zero(sensor_width_, sensor_height_);
    sae_[1] = Eigen::MatrixXd::Zero(sensor_width_, sensor_height_);
    sae_latest_[0] = Eigen::MatrixXd::Zero(sensor_width_, sensor_height_); //only for filter
    sae_latest_[1] = Eigen::MatrixXd::Zero(sensor_width_, sensor_height_);
}

FAHarrisDetector::~FAHarrisDetector() {
}

bool FAHarrisDetector::isFiltered(const dvs_msgs::Event &e)
{
    double et = e.ts.toSec();
    int ex = e.x;
    int ey = e.y;
    bool ep = e.polarity;

    // Update Surface of Active Events
    const int pol = ep ? 1 : 0;
    const int pol_inv = (!ep) ? 1 : 0;
    double & t_last = sae_latest_[pol](ex,ey);
    double & t_last_inv = sae_latest_[pol_inv](ex, ey);

    if ((et > t_last + filter_threshold_) || (t_last_inv > t_last)) //filter_threshold_ = 50 ms
    {
        t_last = et;
        sae_[pol](ex, ey) = et;
        return false;
    }
    else 
    {
        t_last = et;
        return true;
    }
}

bool FAHarrisDetector::isCorner(const dvs_msgs::Event &e)
{
    // Return if too close to the border
    if (e.x < window_size_ or e.x >= sensor_width_ - window_size_ or e.y < window_size_ or e.y >= sensor_height_ - window_size_)
    {
        return false;
    }

    // corner candidate selection and refinement
    if(!isFiltered(e))
    {
        if(isCornerCandidate(e))
        {
            if(isCornerCandidateRefined(e))
            {
                return true;
            }
        }
    }

    return false;
}

bool FAHarrisDetector::addNewEventToTimeSurface(const dvs_msgs::Event &e)
{
    double et = e.ts.toSec();
    int ex = e.x;
    int ey = e.y;
    bool ep = e.polarity;

    // Update Surface of Active Events
    const int pol = ep ? 1 : 0;
    sae_[pol](ex, ey) = et;
}

bool FAHarrisDetector::isCornerWithoutFilter(const dvs_msgs::Event &e)
{
    // Return if too close to the border
    if (e.x < window_size_ or e.x >= sensor_width_ - window_size_ or e.y < window_size_ or e.y >= sensor_height_ - window_size_)
    {
        return false;
    }

    addNewEventToTimeSurface(e);
    if(isCornerCandidate(e))
    {
        if(isCornerCandidateRefined(e))
        {
            return true;
        }
    }

    return false;
}

bool FAHarrisDetector::isCornerCandidate(const dvs_msgs::Event &e) 
{
    int ex = e.x;
    int ey = e.y;
    const int pol = e.polarity ? 1 : 0;

    // Define constant and thresholds
    const int kSmallCircleSize = 16;
    const int kLargeCircleSize = 20;
    const int kSmallMinThresh = 3;
    const int kSmallMaxThresh = 6;
    const int kLargeMinThresh = 4;
    const int kLargeMaxThresh = 8;

    bool is_arc_valid = false;
    // Small Circle exploration
    // Initialize arc from newest element
    double segment_new_min_t = sae_[pol](ex + kSmallCircle_[0][0], ey + kSmallCircle_[0][1]);

    // Left and Right are equivalent to CW and CCW as in the paper
    int arc_right_idx = 0;
    int arc_left_idx;

    // Find newest
    for (int i = 1; i < kSmallCircleSize; i++) 
    {
        const double t = sae_[pol](ex + kSmallCircle_[i][0], ey + kSmallCircle_[i][1]);
        if (t > segment_new_min_t) 
        {
            segment_new_min_t = t;
            arc_right_idx = i; // % End up in the maximum value
        }
    }
    // Shift to the sides of the newest element;
    arc_left_idx = (arc_right_idx - 1 + kSmallCircleSize) % kSmallCircleSize;
    arc_right_idx= (arc_right_idx + 1) % kSmallCircleSize;
    double arc_left_value = sae_[pol](ex + kSmallCircle_[arc_left_idx][0], ey + kSmallCircle_[arc_left_idx][1]);
    double arc_right_value = sae_[pol](ex + kSmallCircle_[arc_right_idx][0], ey + kSmallCircle_[arc_right_idx][1]);
    double arc_left_min_t = arc_left_value;
    double arc_right_min_t = arc_right_value;

    // Expand
    // Initial expand does not require checking
    int iteration = 1; // The arc already contain the maximum
    for (; iteration < kSmallMinThresh; iteration++) 
    {
        // Decide the most promising arc
        if (arc_right_value > arc_left_value) // Right arc
        { 
            if (arc_right_min_t < segment_new_min_t) 
            {
                segment_new_min_t = arc_right_min_t;
            }
            // Expand arc
            arc_right_idx = (arc_right_idx+1) % kSmallCircleSize;
            arc_right_value = sae_[pol](ex + kSmallCircle_[arc_right_idx][0], ey + kSmallCircle_[arc_right_idx][1]);
            if (arc_right_value < arc_right_min_t) // Update minimum of the arc
            { 
                arc_right_min_t = arc_right_value;
            }
        } 
        else // Left arc
        { 
            // Include arc in new segment
            if (arc_left_min_t < segment_new_min_t) 
            {
                segment_new_min_t = arc_left_min_t;
            }

            // Expand arc
            arc_left_idx = (arc_left_idx - 1 + kSmallCircleSize) % kSmallCircleSize;
            arc_left_value = sae_[pol](ex + kSmallCircle_[arc_left_idx][0], ey + kSmallCircle_[arc_left_idx][1]);
            if (arc_left_value < arc_left_min_t) { // Update minimum of the arc
                arc_left_min_t = arc_left_value;
            }
        }
    }
    int newest_segment_size = kSmallMinThresh;

    // Further expand until completion of the circle
    for (; iteration < kSmallCircleSize; iteration++) 
    {
        // Decide the most promising arc
        if (arc_right_value > arc_left_value) // Right arc
        { 
            // Include arc in new segment
            if ((arc_right_value >=  segment_new_min_t)) 
            {
                newest_segment_size = iteration + 1; // Check
                if (arc_right_min_t < segment_new_min_t) 
                {
                    segment_new_min_t = arc_right_min_t;
                }
            }

            // Expand arc
            arc_right_idx = (arc_right_idx + 1) % kSmallCircleSize;
            arc_right_value = sae_[pol](ex + kSmallCircle_[arc_right_idx][0], ey + kSmallCircle_[arc_right_idx][1]);
            if (arc_right_value < arc_right_min_t) // Update minimum of the arc
            { 
                arc_right_min_t = arc_right_value;
            }
        } 
        else // Left arc
        { 
            // Include arc in new segment
            if ((arc_left_value >=  segment_new_min_t)) 
            {
                newest_segment_size = iteration + 1;
                if (arc_left_min_t < segment_new_min_t) 
                {
                    segment_new_min_t = arc_left_min_t;
                }
            }

            // Expand arc
            arc_left_idx = (arc_left_idx - 1 + kSmallCircleSize) % kSmallCircleSize;
            arc_left_value = sae_[pol](ex + kSmallCircle_[arc_left_idx][0], ey+kSmallCircle_[arc_left_idx][1]);
            if (arc_left_value < arc_left_min_t) // Update minimum of the arc
            { 
                arc_left_min_t = arc_left_value;
            }
        }
    }

    if (// Corners with newest segment of a minority of elements in the circle
        // These corners are equivalent to those in Mueggler et al. BMVC17
            (newest_segment_size <= kSmallMaxThresh) ||
        // Corners with newest segment of a majority of elements in the circle
        // This can be commented out to decrease noise at expenses of less repeatibility. If you do, DO NOT forget to comment the equilvent line in the large circle
        ((newest_segment_size >= (kSmallCircleSize - kSmallMaxThresh)) && (newest_segment_size <= (kSmallCircleSize - kSmallMinThresh)))) {
      is_arc_valid = true;
    }

    // Large Circle exploration
    if (is_arc_valid) 
    {
        is_arc_valid = false;

        segment_new_min_t = sae_[pol](ex + kLargeCircle_[0][0], ey + kLargeCircle_[0][1]);
        arc_right_idx = 0;

        // Initialize in the newest element
        for (int i = 1; i < kLargeCircleSize; i++) 
        {
            const double t = sae_[pol](ex + kLargeCircle_[i][0], ey + kLargeCircle_[i][1]);
            if (t > segment_new_min_t) 
            {
                segment_new_min_t = t;
                arc_right_idx = i; // % End up in the maximum value
            }
        }

        // Shift to the sides of the newest elements;
        arc_left_idx = (arc_right_idx - 1 + kLargeCircleSize) % kLargeCircleSize;
        arc_right_idx= (arc_right_idx + 1) % kLargeCircleSize;
        arc_left_value = sae_[pol](ex + kLargeCircle_[arc_left_idx][0],
                                    ey + kLargeCircle_[arc_left_idx][1]);
        arc_right_value = sae_[pol](ex + kLargeCircle_[arc_right_idx][0],
                                    ey + kLargeCircle_[arc_right_idx][1]);
        arc_left_min_t = arc_left_value;
        arc_right_min_t = arc_right_value;

        // Expand
        // Initial expand does not require checking
        iteration = 1;
        for (; iteration < kLargeMinThresh; iteration++) 
        {
            // Decide the most promising arc
            if (arc_right_value > arc_left_value) // Right arc
            { 
                if (arc_right_min_t < segment_new_min_t) 
                {
                    segment_new_min_t = arc_right_min_t;
                }

                // Expand arc
                arc_right_idx = (arc_right_idx + 1) % kLargeCircleSize;
                arc_right_value = sae_[pol](ex + kLargeCircle_[arc_right_idx][0],
                                            ey + kLargeCircle_[arc_right_idx][1]);
                if (arc_right_value < arc_right_min_t) // Update minimum of the arc
                { 
                    arc_right_min_t = arc_right_value;
                }
            } 
            else // Left arc
            { 
                // Include arc in new segment
                if (arc_left_min_t < segment_new_min_t) 
                {
                    segment_new_min_t = arc_left_min_t;
                }

                // Expand arc
                arc_left_idx= (arc_left_idx - 1 + kLargeCircleSize) % kLargeCircleSize;
                arc_left_value = sae_[pol](ex + kLargeCircle_[arc_left_idx][0],
                                            ey + kLargeCircle_[arc_left_idx][1]);

                if (arc_left_value < arc_left_min_t) // Update minimum of the arc
                { 
                    arc_left_min_t = arc_left_value;
                }
            }
        }
        newest_segment_size = kLargeMinThresh;

        // Further expand until completion of the circle
        for (; iteration < kLargeCircleSize; iteration++) 
        {
            // Decide the most promising arc
            if (arc_right_value > arc_left_value)  // Right arc
            {
                // Include arc in new segment
                if ((arc_right_value >=  segment_new_min_t)) 
                {
                    newest_segment_size = iteration + 1;
                    if (arc_right_min_t < segment_new_min_t)
                    {
                        segment_new_min_t = arc_right_min_t;
                    }
                }

                // Expand arc
                arc_right_idx = (arc_right_idx + 1) % kLargeCircleSize;
                arc_right_value = sae_[pol](ex + kLargeCircle_[arc_right_idx][0],
                                            ey + kLargeCircle_[arc_right_idx][1]);
                if (arc_right_value < arc_right_min_t)  // Update minimum of the arc
                {
                    arc_right_min_t = arc_right_value;
                }
            } 
            else // Left arc
            { 
                // Include arc in new segment
                if ((arc_left_value >=  segment_new_min_t)) 
                {
                    newest_segment_size = iteration+1;
                    if (arc_left_min_t < segment_new_min_t) 
                    {
                        segment_new_min_t = arc_left_min_t;
                    }
                }

                // Expand arc
                arc_left_idx = (arc_left_idx - 1 + kLargeCircleSize) % kLargeCircleSize;
                arc_left_value = sae_[pol](ex + kLargeCircle_[arc_left_idx][0],
                                            ey + kLargeCircle_[arc_left_idx][1]);

                if (arc_left_value < arc_left_min_t) // Update minimum of the arc
                { 
                    arc_left_min_t = arc_left_value;
                }
            }
        }

        if (// Corners with newest segment of a minority of elements in the circle
            // These corners are equivalent to those in Mueggler et al. BMVC17
                (newest_segment_size <= kLargeMaxThresh) ||
            // Corners with newest segment of a majority of elements in the circle
            // This can be commented out to decrease noise at expenses of less repeatibility. If you do, DO NOT forget to comment the equilvent line in the small circle
            (newest_segment_size >= (kLargeCircleSize - kLargeMaxThresh) && (newest_segment_size <= (kLargeCircleSize - kLargeMinThresh))) ) {
            is_arc_valid = true;
        }
    }

    return is_arc_valid;
}

bool FAHarrisDetector::isCornerCandidateRefined(const dvs_msgs::Event &e)
{
    // check if queue is full
    double score = harris_threshold_ - 10.;
    if (checkPatch(e))
    {
        // check if current event is a feature
        score = getHarrisScore(e.x, e.y, e.polarity);
    }

    return (score > harris_threshold_);
}

double FAHarrisDetector::getHarrisScore(int img_x, int img_y, bool polarity)
{
    const Eigen::MatrixXi local_frame = getPatch();

    const int l = 2 * window_size_ + 2 - kernel_size_;
    Eigen::MatrixXd dx = Eigen::MatrixXd::Zero(l, l);
    Eigen::MatrixXd dy = Eigen::MatrixXd::Zero(l, l);
    
    for (int x = 0; x < l; x++)
    {
        for (int y = 0; y < l; y++)
        {
            for (int kx = 0; kx < kernel_size_; kx++)
            {
                for (int ky = 0; ky < kernel_size_; ky++)
                {
                    dx(x, y) += local_frame(x + kx, y + ky) * Gx_(kx, ky);
                    dy(x, y) += local_frame(x + kx, y + ky) * Gx_(ky, kx);
                }
            }
        }
    }

    double a = 0., b = 0., d = 0.;
    for (int x = 0; x < l; x++)
    {
        for (int y = 0; y < l; y++)
        {
            a += h_(x, y) * dx(x, y) * dx(x, y);
            b += h_(x, y) * dx(x, y) * dy(x, y);
            d += h_(x, y) * dy(x, y) * dy(x, y);
        }
    }

    const double score = a * d - b * b - 0.04 * (a + d) * (a+d);

    return score;
}

struct LatestEvent
{
    double timestamps;
    int pos_x, pos_y;
};

bool SortByTimestamps(const LatestEvent &e1, const LatestEvent &e2)
{
    return e1.timestamps > e2.timestamps;
}

bool FAHarrisDetector::checkPatch(const dvs_msgs::Event &e)
{
    patch_ = Eigen::MatrixXi::Constant(9, 9, 0);

    vector<LatestEvent> latest_event_;
    const int pol = e.polarity ? 1 : 0;

    // Eigen::MatrixXd local_patch_ = sae_[pol].block<9,9>(e.x - window_size_, e.y - window_size_);

    for (int i = -window_size_; i <= window_size_; i++)
    {
        for (int j = -window_size_; j <= window_size_; j++)
        {
            if(sae_[pol](e.x + i, e.y + j) > 0)
            {
                LatestEvent l_event;
                l_event.timestamps = sae_[pol](e.x + i, e.y + j);
                l_event.pos_x = i + window_size_; // 0-8
                l_event.pos_y = j + window_size_; // 0-8
                latest_event_.push_back(l_event);
            }
        }
    }

    int num_ = latest_event_.size();

    if(num_ < latest_event_local_)
    {
        return false;
    }
    else if(num_ == latest_event_local_)
    {
        for(int n = 0; n < latest_event_local_; n++)
        {
            patch_(latest_event_[n].pos_x, latest_event_[n].pos_y) = 1;
        }

        return true;
    }
    else
    {
        // sort by timestamps
        sort(latest_event_.begin(), latest_event_.end(), SortByTimestamps);

        for(int n = 0; n < latest_event_local_; n++)
        {
            patch_(latest_event_[n].pos_x, latest_event_[n].pos_y) = 1;
        }
        return true;
    }
}

Eigen::MatrixXi FAHarrisDetector::getPatch()
{
    // Eigen::MatrixXi patch_temp_ = patch_.transpose();

    // int rows_ = patch_temp_.rows();
    // int cols_ = patch_temp_.cols();

    // for(int i = 0; i < (rows_ - 1); i++)
    // {
    //     for(int j = 0; j < (cols_ - i - 1); j++)
    //     {
    //         int temp_ = patch_temp_(i, j);
    //         patch_temp_(i, j) = patch_temp_(rows_ - 1 - j, cols_ - 1 - i);
    //         patch_temp_(rows_ - 1 - j, cols_ - 1 - i) = temp_;
    //     }
    // }

    return patch_;
}

int FAHarrisDetector::factorial(int n) const
{
    if (n > 1)
    {
        return n * factorial(n - 1);
    }
    else
    {
        return 1;
    }
}

int FAHarrisDetector::pasc(int k, int n) const
{
    if (k >= 0 && k <= n)
    {
        return factorial(n) / (factorial(n - k) * factorial(k));
    }
    else
    {
        return 0;
    }
}

}
