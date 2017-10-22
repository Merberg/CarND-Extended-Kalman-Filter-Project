#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

class LidarPackage {
 public:
  LidarPackage()
      : timestamp_(0),
        px_(0.0),
        py_(0.0) {
  }

  long long timestamp_;
  float px_;
  float py_;
};

class RadarPackage {
 public:
  RadarPackage()
      : timestamp_(0),
        ro_(0.0),
        theta_(0.0),
        ro_dot_(0.0) {
  }

  long long timestamp_;
  float ro_;
  float theta_;
  float ro_dot_;
};

#endif /* MEASUREMENT_PACKAGE_H_ */
