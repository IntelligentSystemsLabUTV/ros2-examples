/**
 * Custom QoS profiles for action topics.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 *
 * January 12, 2022
 */

#ifndef ADV_QOS_HPP
#define ADV_QOS_HPP

#include <rmw/types.h>

/**
 * Status topic QoS profile.
 */
static const rmw_qos_profile_t status_qos_profile = {
  RMW_QOS_POLICY_HISTORY_KEEP_ALL,
  0,
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false
};


/**
 * Feedback topic QoS profile.
 */
static const rmw_qos_profile_t feedback_qos_profile = {
  RMW_QOS_POLICY_HISTORY_KEEP_ALL,
  0,
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false
};

#endif
