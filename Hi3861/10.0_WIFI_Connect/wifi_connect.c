#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "lwip/netif.h"
#include "lwip/netifapi.h"
#include "lwip/ip4_addr.h"
#include "lwip/api_shell.h"

#include "cmsis_os2.h"
#include "wifi_device.h"
#include "wifiiot_errno.h"
#include "ohos_init.h"

/* ==================== 用户配置：修改为自己的热点信息 ==================== */
#define WIFI_SSID     "iPhone"       // 修改为你的WiFi名称
#define WIFI_PSK      "123456789"       // 修改为你的WiFi密码
#define WIFI_SEC_TYPE  WIFI_SEC_TYPE_PSK  // 加密方式：WIFI_SEC_TYPE_PSK=WPA2-PSK

/* ==================== 超时时间 ==================== */
#define SCAN_TIMEOUT    15   // 扫描超时(秒)
#define CONNECT_TIMEOUT 15   // 连接超时(秒)

/* ==================== 全局标志位 ==================== */
static int g_scanDone    = 0;  // 扫描完成标志
static int g_connected   = 0;  // 连接成功标志
static int g_ssidCount   = 0;  // 扫描到的热点数量
static WifiEvent g_wifiEventHandler = {0};  // WiFi事件回调（必须为全局变量）

/* ==================== WiFi 事件回调函数 ==================== */

// 扫描状态变化回调：state=扫描状态，size=扫描到的热点数量
static void OnScanStateChanged(int state, int size)
{
    if (size > 0) {
        g_ssidCount = size;
        g_scanDone = 1;
    }
    printf("[回调] WiFi扫描状态: state=%d, 热点数量=%d\r\n", state, size);
}

// 连接状态变化回调：state=连接状态，info=连接信息
static void OnConnectionChanged(int state, WifiLinkedInfo *info)
{
    if (info == NULL) {
        printf("[回调] WiFi连接状态: state=%d, info=NULL\r\n", state);
        return;
    }

    if (state == WIFI_STATE_AVALIABLE) {
        g_connected = 1;
        printf("[回调] WiFi已连接! SSID=%s, BSSID=%02X:%02X:%02X:%02X:%02X:%02X, RSSI=%d\r\n",
               info->ssid,
               info->bssid[0], info->bssid[1], info->bssid[2],
               info->bssid[3], info->bssid[4], info->bssid[5],
               info->rssi);
    } else {
        g_connected = 0;
        printf("[回调] WiFi连接断开: state=%d\r\n", state);
    }
}

// 热点模式：有STA接入
static void OnStaJoin(StationInfo *info)
{
    (void)info;
    printf("[回调] 有设备接入热点\r\n");
}

// 热点模式：有STA离开
static void OnStaLeave(StationInfo *info)
{
    (void)info;
    printf("[回调] 有设备离开热点\r\n");
}

// 热点模式状态变化
static void OnHotspotStateChanged(int state)
{
    printf("[回调] 热点状态变化: state=%d\r\n", state);
}

/* ==================== 等待函数 ==================== */

// 等待扫描完成
static void WaitScanResult(void)
{
    int timeout = SCAN_TIMEOUT;
    while (timeout > 0) {
        sleep(1);
        timeout--;
        if (g_scanDone == 1) {
            printf("扫描完成! 等待时间=%ds\r\n", SCAN_TIMEOUT - timeout);
            return;
        }
    }
    printf("扫描超时!\r\n");
}

// 等待连接完成，返回1成功，0失败
static int WaitConnectResult(void)
{
    int timeout = CONNECT_TIMEOUT;
    while (timeout > 0) {
        sleep(1);
        timeout--;
        if (g_connected == 1) {
            printf("连接成功! 等待时间=%ds\r\n", CONNECT_TIMEOUT - timeout);
            return 1;
        }
    }
    printf("连接超时!\r\n");
    return 0;
}

/* ==================== WiFi 初始化（注册事件回调） ==================== */
static void WiFiInit(void)
{
    printf("\r\n========== WiFi 初始化 ==========\r\n");

    g_wifiEventHandler.OnWifiScanStateChanged   = OnScanStateChanged;
    g_wifiEventHandler.OnWifiConnectionChanged  = OnConnectionChanged;
    g_wifiEventHandler.OnHotspotStaJoin         = OnStaJoin;
    g_wifiEventHandler.OnHotspotStaLeave        = OnStaLeave;
    g_wifiEventHandler.OnHotspotStateChanged    = OnHotspotStateChanged;

    WifiErrorCode err = RegisterWifiEvent(&g_wifiEventHandler);
    if (err != WIFI_SUCCESS) {
        printf("注册WiFi事件失败! error=%d\r\n", err);
    } else {
        printf("注册WiFi事件成功\r\n");
    }
}

/* ==================== WiFi 连接主函数 ==================== */
int WifiConnect(const char *ssid, const char *psk)
{
    printf("\r\n========== 开始 WiFi 连接流程 ==========\r\n");
    printf("目标热点: %s\r\n", ssid);

    // 1. 初始化WiFi，注册事件回调
    WiFiInit();

    // 2. 使能WiFi Station模式
    if (EnableWifi() != WIFI_SUCCESS) {
        printf("使能WiFi失败!\r\n");
        return -1;
    }
    printf("WiFi已使能\r\n");

    // 3. 检查WiFi是否激活
    if (IsWifiActive() == 0) {
        printf("WiFi Station未激活!\r\n");
        return -1;
    }
    printf("WiFi Station已激活\r\n");

    // 4. 分配扫描结果缓冲区
    WifiScanInfo *scanInfo = malloc(sizeof(WifiScanInfo) * WIFI_SCAN_HOTSPOT_LIMIT);
    if (scanInfo == NULL) {
        printf("内存分配失败!\r\n");
        return -1;
    }

    // 5. 扫描WiFi热点
    printf("\r\n正在扫描WiFi热点...\r\n");
    do {
        g_ssidCount = 0;
        g_scanDone  = 0;
        Scan();
        WaitScanResult();
    } while (g_scanDone != 1);

    // 6. 获取并打印扫描结果
    unsigned int size = WIFI_SCAN_HOTSPOT_LIMIT;
    GetScanInfoList(scanInfo, &size);

    printf("\r\n========== 扫描到 %d 个WiFi热点 ==========\r\n", g_ssidCount);
    for (int i = 0; i < g_ssidCount; i++) {
        printf("%3d: %-30s 信号强度:%d\r\n", i + 1, scanInfo[i].ssid, scanInfo[i].rssi / 100);
    }
    printf("============================================\r\n\r\n");

    // 7. 在扫描列表中查找目标热点并连接
    int found = 0;
    for (int i = 0; i < g_ssidCount; i++) {
        if (strcmp(ssid, scanInfo[i].ssid) == 0) {
            found = 1;
            printf("找到目标热点: %s (信号强度:%d), 开始连接...\r\n",
                   scanInfo[i].ssid, scanInfo[i].rssi / 100);

            // 配置连接参数
            WifiDeviceConfig config = {0};
            strcpy(config.ssid, scanInfo[i].ssid);
            strcpy(config.preSharedKey, psk);
            config.securityType = WIFI_SEC_TYPE;

            // 添加设备配置
            int netId = 0;
            if (AddDeviceConfig(&config, &netId) != WIFI_SUCCESS) {
                printf("添加WiFi配置失败!\r\n");
                free(scanInfo);
                return -1;
            }

            // 连接热点
            if (ConnectTo(netId) != WIFI_SUCCESS) {
                printf("连接WiFi失败!\r\n");
                free(scanInfo);
                return -1;
            }

            // 等待连接结果
            if (WaitConnectResult() == 1) {
                printf("\r\n*** WiFi连接成功! ***\r\n");
                break;
            } else {
                printf("连接超时!\r\n");
                free(scanInfo);
                return -1;
            }
        }

        if (i == g_ssidCount - 1 && !found) {
            printf("错误: 未找到目标热点 '%s'\r\n", ssid);
            free(scanInfo);
            return -1;
        }
    }

    free(scanInfo);

    // 8. 启动DHCP获取IP地址
    printf("\r\n========== 启动DHCP获取IP ==========\r\n");
    struct netif *netif = netifapi_netif_find("wlan0");
    if (netif) {
        dhcp_start(netif);
        printf("DHCP启动中...\r\n");

        // 等待DHCP完成
        for (;;) {
            if (dhcp_is_bound(netif) == ERR_OK) {
                printf("DHCP获取成功!\r\n");
                netifapi_netif_common(netif, dhcp_clients_info_show, NULL);
                break;
            }
            printf("DHCP获取中...\r\n");
            osDelay(100);
        }
    } else {
        printf("未找到wlan0网络接口!\r\n");
        return -1;
    }

    printf("\r\n========== WiFi连接流程全部完成 ==========\r\n\r\n");
    return 0;
}

/* ==================== 测试任务入口 ==================== */
static void WiFiTask(void)
{
    printf("\r\n");
    printf("==========================================\r\n");
    printf("   OpenHarmony WiFi 连接实验\r\n");
    printf("==========================================\r\n");
    printf("API 函数说明:\r\n");
    printf("  RegisterWifiEvent()   - 注册WiFi事件回调\r\n");
    printf("  EnableWifi()          - 使能WiFi Station模式\r\n");
    printf("  IsWifiActive()        - 检查WiFi是否激活\r\n");
    printf("  Scan()                - 扫描周围WiFi热点\r\n");
    printf("  GetScanInfoList()     - 获取扫描结果列表\r\n");
    printf("  AddDeviceConfig()     - 添加WiFi配置\r\n");
    printf("  ConnectTo()           - 连接到指定热点\r\n");
    printf("  dhcp_start()          - 启动DHCP客户端\r\n");
    printf("  dhcp_is_bound()       - 检查DHCP是否完成\r\n");
    printf("==========================================\r\n\r\n");

    // 调用WiFi连接函数
    WifiConnect(WIFI_SSID, WIFI_PSK);

    // 保持任务运行
    while (1) {
        osDelay(1000);
    }
}

/* ==================== 程序入口（上电自动运行） ==================== */
static void WifiDemo(void)
{
    osThreadAttr_t attr;

    attr.name       = "WiFiTask";
    attr.attr_bits  = 0U;
    attr.cb_mem     = NULL;
    attr.cb_size    = 0U;
    attr.stack_mem  = NULL;
    attr.stack_size = 1024 * 8;
    attr.priority   = 25;

    if (osThreadNew((osThreadFunc_t)WiFiTask, NULL, &attr) == NULL) {
        printf("创建WiFiTask失败!\r\n");
    }
}

APP_FEATURE_INIT(WifiDemo);