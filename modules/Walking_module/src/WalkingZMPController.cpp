/**
 * @file WalkingZMPController.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

// YARP
#include <yarp/os/LogStream.h>

// iDynTree
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/yarp/YARPConversions.h>
#include <iDynTree/yarp/YARPEigenConversions.h>

#include "WalkingZMPController.hpp"
#include "Utils.hpp"

bool WalkingZMPController::initialize(const yarp::os::Searchable& config)
{
    // check if the configuration file is empty
    if(config.isNull())
    {
        yError() << "[initialize] Empty configuration for ZMP controller.";
        return false;
    }

    // set the gain of the CoM controller
    if(!YarpHelper::getDoubleFromSearchable(config, "kCoM", m_kCoM))
    {
        yError() << "[initialize] Unable to get the double from searchable.";
        return false;
    }

    // set the ZMP controller gain
    if(!YarpHelper::getDoubleFromSearchable(config, "kZMP", m_kZMP))
    {
        yError() << "[initialize] Unable to get the double from searchable.";
        return false;
    }

    // set the sampling time
    double samplingTime;
    if(!YarpHelper::getDoubleFromSearchable(config, "sampling_time", samplingTime))
    {
        yError() << "[initialize] Unable to get the double from searchable.";
        return false;
    }

    if(samplingTime < 0)
    {
        yError() << "[initialize] The sampling time has to be a positive number.";
        return false;
    }

    yarp::sig::Vector buffer;
    buffer.resize(2, 0.0);

    // instantiate Integrator object
    m_velocityIntegral = std::make_unique<iCub::ctrl::Integrator>(samplingTime, buffer);

    return true;
}

void WalkingZMPController::setFeedback(const iDynTree::Vector2& zmpFeedback,
                                       const iDynTree::Position& comFeedback)
{
    m_zmpFeedback = zmpFeedback;

    // take only the XY projection of the CoM
    m_comFeedback(0) = comFeedback(0);
    m_comFeedback(1) = comFeedback(1);
}

void WalkingZMPController::setReferenceSignal(const iDynTree::Vector2& zmpDesired,
                                              const iDynTree::Vector2& comPositionDesired,
                                              const iDynTree::Vector2& comVelocityDesired)
{
    m_zmpDesired = zmpDesired;
    m_comPositionDesired = comPositionDesired;
    m_comVelocityDesired = comVelocityDesired;
}

bool WalkingZMPController::evaluateControl()
{
    m_controlEvaluated = false;
    if(m_velocityIntegral == nullptr)
    {
        yError() << "[evaluateControl] The integrator is not initialized.";
        return false;
    }

    // evaluate the control law
    iDynTree::toEigen(m_desiredCoMVelocity) = m_kCoM * (iDynTree::toEigen(m_comPositionDesired) -
                                                        iDynTree::toEigen(m_comFeedback))
                                             -m_kZMP * (iDynTree::toEigen(m_zmpDesired) -
                                                        iDynTree::toEigen(m_zmpFeedback))
                                             +iDynTree::toEigen(m_comVelocityDesired);

    // integrate the velocity
    yarp::sig::Vector desiredCoMVelocityYarp(2);
    yarp::sig::Vector desiredCoMPositionYarp(2);
    iDynTree::toYarp(m_desiredCoMVelocity, desiredCoMVelocityYarp);

    desiredCoMPositionYarp = m_velocityIntegral->integrate(desiredCoMVelocityYarp);

    iDynTree::toiDynTree(desiredCoMPositionYarp, m_controllerOutput);

    m_controlEvaluated = true;
    return true;
}

bool WalkingZMPController::getControllerOutput(iDynTree::Vector2& controllerOutputPosition,
                                               iDynTree::Vector2& controllerOutputVelocity)
{
    if(!m_controlEvaluated)
    {
        yError() << "[getControllerOutput] Please before call evaluateControl() method.";
        return false;
    }

    controllerOutputPosition = m_controllerOutput;
    controllerOutputVelocity = m_desiredCoMVelocity;
    return true;
}

bool WalkingZMPController::reset(const iDynTree::Vector2& initialValue)
{
    if(m_velocityIntegral == nullptr)
    {
        yError() << "[reset] The integrator is not initialized.";
        return false;
    }

    yarp::sig::Vector buffer(2);
    iDynTree::toYarp(initialValue, buffer);

    m_velocityIntegral->reset(buffer);
    return true;
}
