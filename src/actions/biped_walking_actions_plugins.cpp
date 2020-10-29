#include <pal_locomotion_actions_slmc/csv_com_action.h>
#include <pal_locomotion_actions_slmc/csv_walking_action.h>
#include <pal_locomotion_actions_slmc/csv_walking_action_prev.h>
#include <pal_locomotion_actions_slmc/single_leg_stand_action.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(pal_locomotion::CSVCOMAction, pal_locomotion::WalkingActionBase);
PLUGINLIB_EXPORT_CLASS(pal_locomotion::CSVWALKINGAction, pal_locomotion::WalkingActionBase);
PLUGINLIB_EXPORT_CLASS(pal_locomotion::CSVWALKINGActionPrev, pal_locomotion::WalkingActionBase);
PLUGINLIB_EXPORT_CLASS(pal_locomotion::SingleLegStandAction, pal_locomotion::WalkingActionBase);