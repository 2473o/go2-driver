#pragma once

#include<iostream>
#include<vector>
#include "nlohmann/json.hpp"
#include "unitree_api/msg/request.hpp"


#pragma pack(1)
constexpr int32_t ROBOT_SPORT_API_ID_DAMP = 1001;
constexpr int32_t ROBOT_SPORT_API_ID_BALANCESTAND = 1002;
constexpr int32_t ROBOT_SPORT_API_ID_STOPMOVE = 1003;
constexpr int32_t ROBOT_SPORT_API_ID_STANDUP = 1004;
constexpr int32_t ROBOT_SPORT_API_ID_STANDDOWN = 1005;
constexpr int32_t ROBOT_SPORT_API_ID_RECOVERYSTAND = 1006;
constexpr int32_t ROBOT_SPORT_API_ID_EULER = 1007;
constexpr int32_t ROBOT_SPORT_API_ID_MOVE = 1008;
constexpr int32_t ROBOT_SPORT_API_ID_SIT = 1009;
constexpr int32_t ROBOT_SPORT_API_ID_RISESIT = 1010;
constexpr int32_t ROBOT_SPORT_API_ID_SWITCHGAIT = 1011;
constexpr int32_t ROBOT_SPORT_API_ID_TRIGGER = 1012;
constexpr int32_t ROBOT_SPORT_API_ID_BODYHEIGHT = 1013;
constexpr int32_t ROBOT_SPORT_API_ID_FOOTRAISEHEIGHT = 1014;
constexpr int32_t ROBOT_SPORT_API_ID_SPEEDLEVEL = 1015;
constexpr int32_t ROBOT_SPORT_API_ID_HELLO = 1016;
constexpr int32_t ROBOT_SPORT_API_ID_STRETCH = 1017;
constexpr int32_t ROBOT_SPORT_API_ID_TRAJECTORYFOLLOW = 1018;
constexpr int32_t ROBOT_SPORT_API_ID_CONTINUOUSGAIT = 1019;
constexpr int32_t ROBOT_SPORT_API_ID_CONTENT = 1020;
constexpr int32_t ROBOT_SPORT_API_ID_WALLOW = 1021;
constexpr int32_t ROBOT_SPORT_API_ID_DANCE1 = 1022;
constexpr int32_t ROBOT_SPORT_API_ID_DANCE2 = 1023;
constexpr int32_t ROBOT_SPORT_API_ID_GETBODYHEIGHT = 1024;
constexpr int32_t ROBOT_SPORT_API_ID_GETFOOTRAISEHEIGHT = 1025;
constexpr int32_t ROBOT_SPORT_API_ID_GETSPEEDLEVEL = 1026;
constexpr int32_t ROBOT_SPORT_API_ID_SWITCHJOYSTICK = 1027;
constexpr int32_t ROBOT_SPORT_API_ID_POSE = 1028;
constexpr int32_t ROBOT_SPORT_API_ID_SCRAPE = 1029;
constexpr int32_t ROBOT_SPORT_API_ID_FRONTFLIP = 1030;
constexpr int32_t ROBOT_SPORT_API_ID_FRONTJUMP = 1031;
constexpr int32_t ROBOT_SPORT_API_ID_FRONTPOUNCE = 1032;

typedef struct
{
    float timeFromStart;
    float x;
    float y;
    float yaw;
    float vx;
    float vy;
    float vyaw;
} PathPoint;

class SportClient
{
public:
    /*
     * @brief Damp
     * @api: 1001
     */
    void Damp(unitree_api::msg::Request &req);

    /*
     * @brief BalanceStand
     * @api: 1002
     */
    void BalanceStand(unitree_api::msg::Request &req);

    /*
     * @brief StopMove
     * @api: 1003
     */
    void StopMove(unitree_api::msg::Request &req);

    /*
     * @brief StandUp
     * @api: 1004
     */
    void StandUp(unitree_api::msg::Request &req);

    /*
     * @brief StandDown
     * @api: 1005
     */
    void StandDown(unitree_api::msg::Request &req);

    /*
     * @brief RecoveryStand
     * @api: 1006
     */
    void RecoveryStand(unitree_api::msg::Request &req);

    /*
     * @brief Euler
     * @api: 1007
     */
    void Euler(unitree_api::msg::Request &req, float roll, float pitch, float yaw);

    /*
     * @brief Move
     * @api: 1008
     */
    void Move(unitree_api::msg::Request &req, float vx, float vy, float vyaw);

    /*
     * @brief Sit
     * @api: 1009
     */
    void Sit(unitree_api::msg::Request &req);

    /*
     * @brief RiseSit
     * @api: 1010
     */
    void RiseSit(unitree_api::msg::Request &req);

    /*
     * @brief SwitchGait
     * @api: 1011
     */
    void SwitchGait(unitree_api::msg::Request &req, int d);

    /*
     * @brief Trigger
     * @api: 1012
     */
    void Trigger(unitree_api::msg::Request &req);

    /*
     * @brief BodyHeight
     * @api: 1013
     */
    void BodyHeight(unitree_api::msg::Request &req, float height);

    /*
     * @brief FootRaiseHeight
     * @api: 1014
     */
    void FootRaiseHeight(unitree_api::msg::Request &req, float height);

    /*
     * @brief SpeedLevel
     * @api: 1015
     */
    void SpeedLevel(unitree_api::msg::Request &req, int level);

    /*
     * @brief Hello
     * @api: 1016
     */
    void Hello(unitree_api::msg::Request &req);

    /*
     * @brief Stretch
     * @api: 1017
     */
    void Stretch(unitree_api::msg::Request &req);

    /*
     * @brief TrajectoryFollow
     * @api: 1018
     */
    void TrajectoryFollow(unitree_api::msg::Request &req, std::vector<PathPoint> &path);

    /*
     * @brief SwitchJoystick
     * @api: 1027
     */
    void SwitchJoystick(unitree_api::msg::Request &req, bool flag);

    /*
     * @brief ContinuousGait
     * @api: 1019
     */
    void ContinuousGait(unitree_api::msg::Request &req, bool flag);

    /*
     * @brief Wallow
     * @api: 1021
     */
    void Wallow(unitree_api::msg::Request &req);

    /*
     * @brief Content
     * @api: 1020
     */
    void Content(unitree_api::msg::Request &req);

    /*
     * @brief Pose
     * @api: 1028
     */
    void Pose(unitree_api::msg::Request &req, bool flag);

    /*
     * @brief Scrape
     * @api: 1029
     */
    void Scrape(unitree_api::msg::Request &req);

    /*
     * @brief FrontFlip
     * @api: 1030
     */
    void FrontFlip(unitree_api::msg::Request &req);

    /*
     * @brief FrontJump
     * @api: 1031
     */
    void FrontJump(unitree_api::msg::Request &req);

    /*
     * @brief FrontPounce
     * @api: 1032
     */
    void FrontPounce(unitree_api::msg::Request &req);

    /*
     * @brief Dance1
     * @api: 1022
     */
    void Dance1(unitree_api::msg::Request &req);

    /*
     * @brief Dance2
     * @api: 1023
     */
    void Dance2(unitree_api::msg::Request &req);
};


inline void SportClient::Damp(unitree_api::msg::Request &req)
{
    req.header.identity.api_id = ROBOT_SPORT_API_ID_DAMP;
}

inline void SportClient::BalanceStand(unitree_api::msg::Request &req)
{
    req.header.identity.api_id = ROBOT_SPORT_API_ID_BALANCESTAND;
}

inline void SportClient::StopMove(unitree_api::msg::Request &req)
{
    req.header.identity.api_id = ROBOT_SPORT_API_ID_STOPMOVE;
}

inline void SportClient::StandUp(unitree_api::msg::Request &req)
{
    req.header.identity.api_id = ROBOT_SPORT_API_ID_STANDUP;
}

inline void SportClient::StandDown(unitree_api::msg::Request &req)
{
    req.header.identity.api_id = ROBOT_SPORT_API_ID_STANDDOWN;
}

inline void SportClient::RecoveryStand(unitree_api::msg::Request &req)
{
    req.header.identity.api_id = ROBOT_SPORT_API_ID_RECOVERYSTAND;
}

inline void SportClient::Euler(unitree_api::msg::Request &req, float roll, float pitch, float yaw)
{
    nlohmann::json js;
    js["x"] = roll;
    js["y"] = pitch;
    js["z"] = yaw;
    req.parameter = js.dump();
    req.header.identity.api_id = ROBOT_SPORT_API_ID_EULER;
}

inline void SportClient::Move(unitree_api::msg::Request &req, float vx, float vy, float vyaw)
{
    nlohmann::json js;
    js["x"] = vx;
    js["y"] = vy;
    js["z"] = vyaw;
    req.parameter = js.dump();
    req.header.identity.api_id = ROBOT_SPORT_API_ID_MOVE;
}

inline void SportClient::Sit(unitree_api::msg::Request &req)
{
    req.header.identity.api_id = ROBOT_SPORT_API_ID_SIT;
}

inline void SportClient::RiseSit(unitree_api::msg::Request &req)
{
    req.header.identity.api_id = ROBOT_SPORT_API_ID_RISESIT;
}

inline void SportClient::SwitchGait(unitree_api::msg::Request &req, int d)
{
    nlohmann::json js;
    js["data"] = d;
    req.header.identity.api_id = ROBOT_SPORT_API_ID_SWITCHGAIT;
    req.parameter = js.dump();
}

inline void SportClient::Trigger(unitree_api::msg::Request &req)
{
    req.header.identity.api_id = ROBOT_SPORT_API_ID_TRIGGER;
}

inline void SportClient::BodyHeight(unitree_api::msg::Request &req, float height)
{
    nlohmann::json js;
    js["data"] = height;
    req.parameter = js.dump();
    req.header.identity.api_id = ROBOT_SPORT_API_ID_BODYHEIGHT;
}

inline void SportClient::FootRaiseHeight(unitree_api::msg::Request &req, float height)
{
    nlohmann::json js;
    js["data"] = height;
    req.parameter = js.dump();
    req.header.identity.api_id = ROBOT_SPORT_API_ID_FOOTRAISEHEIGHT;
}

inline void SportClient::SpeedLevel(unitree_api::msg::Request &req, int level)
{
    nlohmann::json js;
    js["data"] = level;
    req.parameter = js.dump();
    req.header.identity.api_id = ROBOT_SPORT_API_ID_SPEEDLEVEL;
}

inline void SportClient::Hello(unitree_api::msg::Request &req)
{
    req.header.identity.api_id = ROBOT_SPORT_API_ID_HELLO;
}

inline void SportClient::Stretch(unitree_api::msg::Request &req)
{
    req.header.identity.api_id = ROBOT_SPORT_API_ID_STRETCH;
}

inline void SportClient::TrajectoryFollow(unitree_api::msg::Request &req, std::vector<PathPoint> &path)
{
    nlohmann::json js_path;
    req.header.identity.api_id = ROBOT_SPORT_API_ID_TRAJECTORYFOLLOW;
    for (int i = 0; i < 30; i++)
    {
        nlohmann::json js_point;
        js_point["t_from_start"] = path[i].timeFromStart;
        js_point["x"] = path[i].x;
        js_point["y"] = path[i].y;
        js_point["yaw"] = path[i].yaw;
        js_point["vx"] = path[i].vx;
        js_point["vy"] = path[i].vy;
        js_point["vyaw"] = path[i].vyaw;
        js_path.push_back(js_point);
    }
    req.parameter =js_path.dump();
}

inline void SportClient::SwitchJoystick(unitree_api::msg::Request &req, bool flag)
{
    nlohmann::json js;
    js["data"] = flag;
    req.parameter = js.dump();
    req.header.identity.api_id = ROBOT_SPORT_API_ID_SWITCHJOYSTICK;
}

inline void SportClient::ContinuousGait(unitree_api::msg::Request &req, bool flag)
{
    nlohmann::json js;
    js["data"] = flag;
    req.parameter = js.dump();
    req.header.identity.api_id = ROBOT_SPORT_API_ID_CONTINUOUSGAIT;
}

inline void SportClient::Wallow(unitree_api::msg::Request &req)
{
    req.header.identity.api_id = ROBOT_SPORT_API_ID_WALLOW;
}

inline void SportClient::Content(unitree_api::msg::Request &req)
{
    req.header.identity.api_id = ROBOT_SPORT_API_ID_CONTENT;
}

inline void SportClient::Pose(unitree_api::msg::Request &req, bool flag)
{
    nlohmann::json js;
    js["data"] = flag;
    req.parameter = js.dump();
    req.header.identity.api_id = ROBOT_SPORT_API_ID_POSE;
}

inline void SportClient::Scrape(unitree_api::msg::Request &req)
{
    req.header.identity.api_id = ROBOT_SPORT_API_ID_SCRAPE;
}

inline void SportClient::FrontFlip(unitree_api::msg::Request &req)
{
    req.header.identity.api_id = ROBOT_SPORT_API_ID_FRONTFLIP;
}

inline void SportClient::FrontJump(unitree_api::msg::Request &req)
{
    req.header.identity.api_id = ROBOT_SPORT_API_ID_FRONTJUMP;
}

inline void SportClient::FrontPounce(unitree_api::msg::Request &req)
{
    req.header.identity.api_id = ROBOT_SPORT_API_ID_FRONTPOUNCE;
}

inline void SportClient::Dance1(unitree_api::msg::Request &req)
{
    req.header.identity.api_id = ROBOT_SPORT_API_ID_DANCE1;
}

inline void SportClient::Dance2(unitree_api::msg::Request &req)
{
    req.header.identity.api_id = ROBOT_SPORT_API_ID_DANCE2;
}