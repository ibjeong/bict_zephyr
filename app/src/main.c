/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <ff.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/fs/fs.h>
#include <zephyr/logging/log.h>
#include <zephyr/storage/disk_access.h>

#include <zephyr/sys/cbprintf.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/ring_buffer.h>

LOG_MODULE_REGISTER(main);

static void bt_device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type, struct net_buf_simple *ad);

// main uses this to buffer items
#define WRITE_BUFFER_SIZE (512 * 10) // write at once up to 10 sectors
static uint8_t write_buffer[WRITE_BUFFER_SIZE + 1]; // +1 for EOL

K_MUTEX_DEFINE(rbuf_mutex);
K_CONDVAR_DEFINE(rbuf_cv);
RING_BUF_ITEM_DECLARE_POW2(rbuf, (9 + 5)); // 512 = 2^9, 2^5=32 sectors

static FATFS fat_fs;
static struct fs_mount_t mp = {
  .type = FS_FATFS,
  .mnt_point = "/SD:",
  .fs_data = &fat_fs,
};

void main(void)
{
  int err;

  // Initialize Bluetooth
  if((err = bt_enable(NULL))) {
    LOG_ERR("No Bluetooth");
    return;
  }

  if((err = fs_mount(&mp))) {
    LOG_ERR("err: fs_mount: %d\n", err);
    return;
  }

  // file open
  struct fs_file_t file;
  fs_file_t_init(&file);
  if((err = fs_open(&file, "/SD:/log.csv", FS_O_WRITE | FS_O_CREATE))) {
    LOG_ERR("err: fs_open: %d\n", err);
    return;
  }

  if((err = bt_le_scan_start(BT_LE_SCAN_PARAM(BT_LE_SCAN_TYPE_PASSIVE,  \
                                              BT_HCI_LE_SCAN_FILTER_DUP_DISABLE, \
                                              BT_GAP_SCAN_FAST_INTERVAL, \
                                              BT_GAP_SCAN_FAST_WINDOW),
                             bt_device_found))) {
    LOG_ERR("err: bt_le_scan_start: %d\n", err);
    return;
  }

  while(1) {
    k_mutex_lock(&rbuf_mutex, K_FOREVER);

    uint32_t size;
    while((size = ring_buf_size_get(&rbuf)) < 512) {
      k_condvar_wait(&rbuf_cv, &rbuf_mutex, K_FOREVER);
    }

    size_t ret = ring_buf_get(&rbuf, write_buffer, MIN((size / 512) * 512, WRITE_BUFFER_SIZE));

    k_mutex_unlock(&rbuf_mutex);

    fs_write(&file, write_buffer, ret);
    fs_sync(&file);

    write_buffer[ret] = '\0';
    printk("%s", write_buffer);
  }
}

void bt_device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
                     struct net_buf_simple *ad)
{
  static const uint8_t uuid[] = {0xb9, 0x40, 0x7f, 0x30, 0xf5, 0xf8, 0x46, 0x6e, 0xaf, 0xf9, 0x25, 0x55, 0x6b, 0x57, 0xfe, 0x6d,};
  static uint8_t msg_buffer[512];

  if((ad->len < 30) || memcmp(&ad->data[0x09], uuid, sizeof(uuid))) {
    return;
  }

  uint16_t major = (ad->data[0x19] << 8) | ad->data[0x1a];
  uint16_t minor = (ad->data[0x1b] << 8) | ad->data[0x1c];
  int8_t tx = ad->data[0x1d];

  if ((major >= 310) && (major <= 312)) {
    int len = snprintfcb(msg_buffer, sizeof(msg_buffer),
                         "RSSI,%lld,%04x,%04x,%d,%d\r\n",
                         k_uptime_get(),
                         major,
                         minor,
                         tx,
                         rssi);

    k_mutex_lock(&rbuf_mutex, K_FOREVER);
    ring_buf_put(&rbuf, msg_buffer, len);
    k_condvar_signal(&rbuf_cv);
    k_mutex_unlock(&rbuf_mutex);
  }
}
