#ifndef VERTEX_TIMEDIFF_H
#define VERTEX_TIMEDIFF_H

#include <g2o/config.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/hyper_graph_action.h>

#include <Eigen/Core>

namespace nav2_teb_controller
{

/**
  * @class VertexTimeDiff
  * @brief This class stores and wraps a time difference \f$ \Delta T \f$ into a vertex that can be optimized via g2o
  */
class VertexTimeDiff : public g2o::BaseVertex<1, double>
{
public:

  /**
    * @brief Default constructor
    * @param fixed if \c true, this vertex is considered fixed during optimization [default: \c false]
    */  
  VertexTimeDiff(bool fixed = false)
  {
    setToOriginImpl();
    setFixed(fixed);
  }
  
  /**
    * @brief Construct the TimeDiff vertex with a value
    * @param dt time difference value of the vertex
    * @param fixed if \c true, this vertex is considered fixed during optimization [default: \c false]
    */  
  VertexTimeDiff(double dt, bool fixed = false)
  {
    _estimate = dt;
    setFixed(fixed);
  }

  /**
    * @brief Access the timediff value of the vertex
    * @see estimate
    * @return reference to dt
    */ 
  double& dt() {return _estimate;}
  
  /**
    * @brief Access the timediff value of the vertex (read-only)
    * @see estimate
    * @return const reference to dt
    */ 
  [[nodiscard]] const double& dt() const {return _estimate;}
  
  /**
    * @brief Set the underlying TimeDiff estimate \f$ \Delta T \f$ to default.
    */ 
  void setToOriginImpl() override
  {
    _estimate = 0.1;
  }

  /**
    * @brief Define the update increment \f$ \Delta T_{k+1} = \Delta T_k + update \f$.
    * A simple addition implements what we want.
    * @param update increment that should be added to the previous esimate
    */ 
  void oplusImpl(const double* update) override
  {
      _estimate += *update;
  }

  /**
    * @brief Read an estimate of \f$ \Delta T \f$ from an input stream
    * @param is input stream
    * @return always \c true
    */ 
  bool read(std::istream& is) override
  {
    is >> _estimate;
    return true;
  }

  /**
    * @brief Write the estimate \f$ \Delta T \f$ to an output stream
    * @param os output stream
    * @return \c true if the export was successful, otherwise \c false
    */ 
  bool write(std::ostream& os) const override
  {
    os << estimate();
    return os.good();
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}

#endif
