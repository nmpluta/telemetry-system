/*
 * Copyright (c) 2022 Natalia Pluta
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file sd_threads.c
 * @brief SD card logging related threads
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>

#include <zephyr/storage/disk_access.h>
#include <zephyr/logging/log.h>
#include <zephyr/fs/fs.h>
#include <ff.h>

#include "sd_threads.h"

#define SD_LOGGING_THREAD_STACK_SIZE 2048
#define SD_LOGGING_THREAD_PRIORITY   2
#define SLEEP_TIME                   K_MSEC(50)

#define SD_LOGS 100

#define FATFS_MNTP "/SD:"
#define FILE       FATFS_MNTP "/test.txt"
#define DIR        FATFS_MNTP "/testdir"
#define DIR_FILE   FATFS_MNTP "/testdir/testfile.txt"

LOG_MODULE_REGISTER(sd_logging, LOG_LEVEL_DBG);

typedef enum
{
    SD_LOGGING_DATA,
    SD_CLOSING_FILE,
    SD_UNMOUNTED,
} sd_state_t;

static struct fs_dir_t  dirp;
static struct fs_file_t filep;
char *                  p_obd2_string_data;
static FATFS            fat_fs;

static struct fs_mount_t fatfs_mount = {
    .type      = FS_FATFS,
    .mnt_point = FATFS_MNTP,
    .fs_data   = &fat_fs,
};


void sd_file_create(void);
void sd_file_write(char * p_string_data, size_t string_size);
void sd_file_save(void);
size_t obd2_string_frame_create(struct can_frame * frame, char * p_string_data);
static void lsdir(const char * path);

static const char * disk_mount_pt = FATFS_MNTP;

K_THREAD_STACK_DEFINE(sd_logging_stack, SD_LOGGING_THREAD_STACK_SIZE);
struct k_thread sd_logging_data;
k_tid_t         sd_logging_tid;

CAN_MSGQ_DEFINE(sd_msgq, 10);
extern void obd2_frame_print(struct can_frame * frame);

/**
 * It initializes the SD card
 */
void sd_disk_init(void)
{
    /* raw disk i/o */
    do
    {
        static const char * disk_pdrv = "SD";
        uint64_t            memory_size_mb;
        uint32_t            block_count;
        uint32_t            block_size;

        if (disk_access_init(disk_pdrv) != 0)
        {
            LOG_ERR("Storage init ERROR!.");
            break;
        }

        if (disk_access_ioctl(disk_pdrv, DISK_IOCTL_GET_SECTOR_COUNT, &block_count))
        {
            LOG_ERR("Unable to get sector count.");
            break;
        }
        LOG_INF("Block count %u.", block_count);

        if (disk_access_ioctl(disk_pdrv, DISK_IOCTL_GET_SECTOR_SIZE, &block_size))
        {
            LOG_ERR("Unable to get sector size.");
            break;
        }
        LOG_INF("Sector size %u", block_size);

        memory_size_mb = (uint64_t)block_count * block_size;
        LOG_INF("Memory Size(MB) %u.\n", (uint32_t)(memory_size_mb >> 20));
    } while (0);
}

/**
 * It creates a file, writes the received CAN frames to it, and closes the file
 * 
 * @param arg1 The first argument to pass to the thread.
 * @param arg2 The size of the message queue.
 * @param arg3 The third argument to the thread.
 */
void sd_logging_thread(void * arg1, void * arg2, void * arg3)
{
    LOG_INF("Initialization of sd_logging_thread.");

    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    sd_file_create();

    struct can_frame frame;
    sd_state_t       sd_state   = SD_LOGGING_DATA;
    uint16_t         sd_log_idx = 0;
    while (1)
    {
        k_msgq_get(&sd_msgq, &frame, K_FOREVER);
        printf("[SD] RECEIVED:\n");
        obd2_frame_print(&frame);

        switch (sd_state)
        {
            case SD_LOGGING_DATA:
                if (sd_log_idx < SD_LOGS)
                {
                    size_t obd2_string_size = obd2_string_frame_create(&frame, p_obd2_string_data);
                    sd_file_write(p_obd2_string_data, obd2_string_size);
                    sd_log_idx++;
                }
                else
                {
                    sd_state = SD_CLOSING_FILE;
                }
                break;

            case SD_CLOSING_FILE:
                sd_file_save();
                sd_state = SD_UNMOUNTED;
                break;

            case SD_UNMOUNTED:
                break;

            default:
                break;
        }
        k_sleep(SLEEP_TIME);
    }
}

/**
 * It creates a thread that will be used to log data to the SD card
 */
void sd_threads_init(void)
{
    sd_logging_tid = k_thread_create(&sd_logging_data,
                                     sd_logging_stack,
                                     K_THREAD_STACK_SIZEOF(sd_logging_stack),
                                     sd_logging_thread,
                                     NULL,
                                     NULL,
                                     NULL,
                                     SD_LOGGING_THREAD_PRIORITY,
                                     0,
                                     K_NO_WAIT);
    if (!sd_logging_tid)
    {
        LOG_ERR("Failed spawning can state thread.");
    }
}

/**
 * It mounts the file system and creates a file
 */
void sd_file_create(void)
{
    int ret;
    (void)ret;

    fatfs_mount.mnt_point = disk_mount_pt;

    ret = fs_mount(&fatfs_mount);
    if (0 != ret)
    {
        LOG_ERR("Failed to mount file system.\n");
    }
    else
    {
        LOG_INF("Mounted file system.\n");
    }

    struct fs_statvfs stat;

    /* Verify fs_statvfs() */
    ret = fs_statvfs(FATFS_MNTP, &stat);
    if (0 != ret)
    {
        LOG_ERR("Error getting volume stats [%d].", ret);
    }

    LOG_INF("Optimal transfer block size   = %lu.", stat.f_bsize);
    LOG_INF("Allocation unit size          = %lu.", stat.f_frsize);
    LOG_INF("Volume size in f_frsize units = %lu.", stat.f_blocks);
    LOG_INF("Free space in f_frsize units  = %lu.\n", stat.f_bfree);

    fs_dir_t_init(&dirp);

    /* Verify fs_opendir() */
    ret = fs_opendir(&dirp, fatfs_mount.mnt_point);
    if (0 != ret)
    {
        LOG_ERR("Failed to open dir %s [%d].", fatfs_mount.mnt_point, ret);
    }
    else
    {
        LOG_INF("Opened dir %s.", fatfs_mount.mnt_point);
    }

    lsdir(disk_mount_pt);

    fs_file_t_init(&filep);

    ret = fs_open(&filep, FILE, FS_O_RDWR | FS_O_CREATE);
    if (0 != ret)
    {
        LOG_ERR("Failed to open file: %s.\n", FILE);
    }
    else
    {
        LOG_INF("Opened file %s.\n", FILE);
    }

    ret = fs_seek(&filep, 0, FS_SEEK_SET);
    if (ret)
    {
        LOG_ERR("fs_seek failed [%d]", ret);
        fs_close(&filep);
    }
}

/**
 * It writes the data to the file
 * 
 * @param p_string_data The data to be written to the file.
 * @param string_size The size of the string to be written to the file.
 */
void sd_file_write(char * p_string_data, size_t string_size)
{
    int ret;
    (void)ret;
    ssize_t brw;

    ret = fs_seek(&filep, 0, FS_SEEK_CUR);
    if (ret)
    {
        LOG_ERR("fs_seek failed [%d]", ret);
        fs_close(&filep);
    }

    // LOG_INF("Data written:\"%s\"", p_string_data);

    brw = fs_write(&filep, p_string_data, string_size);
    if (brw < 0)
    {
        LOG_ERR("Failed writing to file [%zd]", brw);
    }

    if (brw < string_size)
    {
        LOG_ERR("Unable to complete write. Volume full.");
        LOG_INF("Number of bytes written: [%zd].", brw);
    }
}

/**
 * It takes a CAN frame and a string buffer, and it writes the CAN frame to the string buffer in a
 * human readable format
 *
 * @param frame The CAN frame to be converted to a string.
 * @param p_string_data A pointer to the string buffer where the data will be stored.
 *
 * @return The number of characters written to the string.
 */
size_t obd2_string_frame_create(struct can_frame * frame, char * p_string_data)
{
    size_t char_nums = sprintf(p_string_data,
                               "%03x, %02x %02x %02x %02x %02x %02x %02x %02x\n",
                               frame->id,
                               frame->data[0],
                               frame->data[1],
                               frame->data[2],
                               frame->data[3],
                               frame->data[4],
                               frame->data[5],
                               frame->data[6],
                               frame->data[7]);
    return char_nums;
}

/**
 * It opens a file, writes to it, syncs it, closes it, lists the directory, closes the directory, and
 * unmounts the file system
 */
void sd_file_save(void)
{
    int ret;
    (void)ret;
    ret = fs_sync(&filep);
    if (0 != ret)
    {
        LOG_ERR("Failed to sync file: %s.\n", FILE);
    }
    else
    {
        LOG_INF("Synced file %s.\n", FILE);
    }

    ret = fs_close(&filep);
    if (0 != ret)
    {
        LOG_ERR("Failed to close file: %s.", FILE);
    }
    else
    {
        LOG_INF("Closed file %s.", FILE);
    }

    lsdir(disk_mount_pt);

    ret = fs_closedir(&dirp);
    if (ret)
    {
        LOG_ERR("Failed to close dir %s [%d].", fatfs_mount.mnt_point, ret);
    }
    else
    {
        LOG_INF("Closed dir %s.", fatfs_mount.mnt_point);
    }

    ret = fs_unmount(&fatfs_mount);
    if (0 != ret)
    {
        LOG_ERR("Failed to unmount file system.");
    }
    else
    {
        LOG_INF("Unmounted file system.");
    }
}

/**
 * It opens a directory, reads its content and prints it to the console
 * 
 * @param path The path to the directory to list.
 */
static void lsdir(const char * path)
{
    int                     ret;
    struct fs_dir_t         dirp;
    static struct fs_dirent entry;

	fs_dir_t_init(&dirp);

	/* Verify fs_opendir() */
	ret = fs_opendir(&dirp, path);
	if (ret) {
		LOG_ERR("Error opening dir %s [%d]\n", path, ret);
	}

    LOG_INF("Listing dir %s ...", path);
    for (;;)
    {
        /* Verify fs_readdir() */
        ret = fs_readdir(&dirp, &entry);

        /* entry.name[0] == 0 means end-of-dir */
        if (ret || entry.name[0] == 0)
        {
            break;
        }

        if (entry.type == FS_DIR_ENTRY_DIR)
        {
            LOG_INF("[DIR ] %s.", entry.name);
        }
        else
        {
            LOG_INF("[FILE] %s (size = %zu).", entry.name, entry.size);
        }
    }

	/* Verify fs_closedir() */
	fs_closedir(&dirp);
}
