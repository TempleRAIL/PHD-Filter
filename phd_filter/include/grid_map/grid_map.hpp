#ifndef GRID_MAP_HPP
#define GRID_MAP_HPP

#include <string>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

namespace phd {

class GridMap {
public:
  GridMap(double origin_x, double width, double origin_y, double height,
          double meters_per_pixel, double max_value) : origin_x_(origin_x),
      origin_y_(origin_y), meters_per_pixel_(meters_per_pixel),
      max_value_(max_value) {
    width_ = ceil(width / meters_per_pixel_);
    height_ = ceil(height / meters_per_pixel_);
    ros_grid_.reset(new nav_msgs::OccupancyGrid);
    ros_grid_->header.frame_id = "/map";
    ros_grid_->data.resize(width_ * height_);
    ros_grid_->info.resolution = meters_per_pixel_;
    ros_grid_->info.width = width_;
    ros_grid_->info.height = height_;
    ros_grid_->info.origin.position.x = origin_x_;
    ros_grid_->info.origin.position.y = origin_y_;
    ros_grid_->info.origin.orientation.w = 1.0;

    grid_ = new double[width_ * height_];
  }

  GridMap(const nav_msgs::OccupancyGrid& grid, double max_value = 1e6) :
      origin_x_(grid.info.origin.position.x),
      origin_y_(grid.info.origin.position.y),
      meters_per_pixel_(grid.info.resolution),
      width_(grid.info.width), height_(grid.info.height),
      max_value_(max_value), ros_grid_(new nav_msgs::OccupancyGrid(grid)) {
    grid_ = new double[width_ * height_];
    assert(width_ * height_ == ros_grid_->data.size());
    for (int i = 0; i < ros_grid_->data.size(); ++i) {
      grid_[i] = ros_grid_->data[i];
    }
  }

  ~GridMap() {
    delete[] grid_;
  }

  double metersPerPixel() const { return meters_per_pixel_; }
  int width() const { return width_; }
  int height() const { return height_; }
  double originX() const { return origin_x_; }
  double originY() const { return origin_y_; }

  void getCoord(int xi, int yi, double *x, double *y) const {
    *x = (xi + 0.5) * meters_per_pixel_ + origin_x_;
    *y = (yi + 0.5) * meters_per_pixel_ + origin_y_;
  }

  void getCoord(int ind, double *x, double *y) const {
    int xi, yi;
    getSubscript(ind, &xi, &yi);
    getCoord(xi, yi, x, y);
  }

  void getSubscript(double x, double y, int *xi, int *yi) const {
    *xi = (x - origin_x_) / meters_per_pixel_;
    *yi = (y - origin_y_) / meters_per_pixel_;
  }

  void getSubscript(int ind, int *xi, int *yi) const {
    *xi = ind % width_;
    *yi = ind / width_;
  }

  void getIndex(int xi, int yi, int* ind) const {
    *ind = yi * width_ + xi;
  }

  void getIndex(double x, double y, int* ind) const {
    int xi, yi;
    getSubscript(x, y, &xi, &yi);
    getIndex(xi, yi, ind);
  }

  bool valid(int xi, int yi) const {
    return !(xi >= width_ || yi >= height_ || xi < 0 || yi < 0);
  }

  double get(int xi, int yi) const {
    if (valid(xi, yi)) {
      int ind;
      getIndex(xi, yi, &ind);
      return grid_[ind];
    } else {
      return 0;
    }
  }

  void set(int xi, int yi, double val) {
    if (valid(xi, yi)) {
      int ind;
      getIndex(xi, yi, &ind);
      grid_[ind] = val;
      ros_grid_->data[ind] = std::min(100, static_cast<int>(100.0 * grid_[ind] / max_value_));
    } else {
      ROS_WARN("Setting coordinates outside map");
    }
  }

  void fill(double val) {
    for (int i = 0; i < width_ * height_; ++i) {
      grid_[i] = val;
      ros_grid_->data[i] = std::min(100, static_cast<int>(100.0 * grid_[i] / max_value_));
    }
  }

  void setFrameId(const std::string &frame) {
    ros_grid_->header.frame_id = frame;
  }

  const nav_msgs::OccupancyGrid& occGrid() const {
    return *ros_grid_;
  }

private:
  GridMap() {};
  GridMap(const GridMap &map);
  void operator=(const GridMap&);

  double origin_x_, origin_y_; // Lower left coordinate of map in meters
  double meters_per_pixel_;
  int width_, height_; // Size of grid_ in pixels
  double max_value_; // Value to saturate plots
  double *grid_; // Expected number of targets
  boost::scoped_ptr<nav_msgs::OccupancyGrid> ros_grid_;
};

} // end namespace

#endif
