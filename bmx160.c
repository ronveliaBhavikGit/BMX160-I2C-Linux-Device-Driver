#include <linux/init.h>             // Provides macros like __init, __exit used to mark initialization and cleanup functions
#include <linux/module.h>           // Declares module metadata macros like MODULE_LICENSE, MODULE_AUTHOR, and provides functions like module_init() and module_exit().
#include <linux/uaccess.h>          // Contains functions to safely transfer data between user space and kernel space.
#include <linux/i2c.h>              // Provides the I2C subsystem core functionality — i2c_client, i2c_driver, i2c_transfer(), etc.
#include <linux/device.h>           // Contains APIs for managing struct device, device creation, sysfs, class, etc
#include <linux/mod_devicetable.h>  // Contains device ID tables (e.g., i2c_device_id, of_device_id).
#include <linux/delay.h>            // Provides delay functions like msleep(), udelay(), mdelay()
#include <linux/kthread.h>          // Provides support for creating and managing kernel threads using kthread_run(), kthread_stop()

/**********************************************************************/

#define DEVICE_NAME "bmx160i2c"

#define BMX160_CHIP_ID_REG                  0x00
#define BMX160_CHIP_ID                      0xD8
#define BMX160_CMD_REG                      0x7E
#define BMX160_SOFT_RESET_CMD               0xB6


#define BMX160_ACCEL_NORMAL_MODE_CMD        0x11    // 0x12 for low power
#define BMX160_GYRO_NORMAL_MODE_CMD         0x15    // 0x17 for low power
#define BMX160_MEGNETO_NORMAL_MODE_CMD      0x1B    // 0x1B for low power
#define BMX160_ACCEL_DATA_ADDR              0x12
#define BMX160_GYRO_DATA_ADDR               0x0C
#define BMX160_MEGNETO_DATA_ADDR            0x04

static const struct i2c_device_id bmx160_id[] = {
    {"bmx160", 0},
    {}
};
MODULE_DEVICE_TABLE(i2c, bmx160_id);

/**********************************************************************/

static int __init bmx160_module_init(void);
static void __exit bmx160_module_exit(void);

static int bmx160_probe(struct i2c_client *client);
static void bmx160_remove(struct i2c_client *client);

static int bmx160_read_accel_data(struct i2c_client *client);
static int bmx160_read_gyro_data(struct i2c_client *client);
static int bmx160_read_megneto_data(struct i2c_client *client);

static int bmx160_polling_thread(void *data);

static struct i2c_driver bmx160_driver;
static struct task_struct *bmx160_thread;
static bool thread_run =true;
/**********************************************************************/

static int bmx160_probe(struct i2c_client *client)
{
    int ret;
    u8 chip_id;

    dev_info(&client->dev, "Probing BMX160 sensor...\n");

    // Step 1: Read CHIP ID from register 0x00
    ret = i2c_smbus_read_byte_data(client, BMX160_CHIP_ID_REG);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to read chip ID register\n");
        return ret;
    }

    chip_id = (u8)ret;
    dev_info(&client->dev, "BMX160 Chip ID read: 0x%02X\n", chip_id);

    // Step 2: Verify CHIP ID
    if (chip_id != BMX160_CHIP_ID) {
        dev_err(&client->dev, "Unexpected Chip ID: 0x%02X (Expected 0x%02X)\n", chip_id, BMX160_CHIP_ID);
        return -ENODEV;
    }

    dev_info(&client->dev, "BMX160 sensor detected and ready!\n");

    // Soft reset
    ret = i2c_smbus_write_byte_data(client, BMX160_CMD_REG, BMX160_SOFT_RESET_CMD);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to write soft reset command\n");
        return ret;
    }
    msleep(100); // Wait for reset to complete

    // Accelerometer normal mode
    ret = i2c_smbus_write_byte_data(client, BMX160_CMD_REG, BMX160_ACCEL_NORMAL_MODE_CMD);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to set accelerometer normal mode\n");
        return ret;
    }
    msleep(50);

    // Gyroscope normal mode
    ret = i2c_smbus_write_byte_data(client, BMX160_CMD_REG, BMX160_GYRO_NORMAL_MODE_CMD);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to set gyroscope normal mode\n");
        return ret;
    }
    msleep(50);

    // Megnetometer normal mode (Since it is an external sensor BMM150, we need some more configurations)
    ret = i2c_smbus_write_byte_data(client, BMX160_CMD_REG, BMX160_MEGNETO_NORMAL_MODE_CMD);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to set megnetometer normal mode\n");
        return ret;
    }
    msleep(50);

    dev_info(&client->dev, "BMX160 initialized with accel, gyro and megneto enabled\n");\

    bmx160_thread = kthread_run(bmx160_polling_thread, client, "bmx160_thread");
    if(IS_ERR(bmx160_thread))
    {
        dev_err(&client->dev, "Failed to create polling thread\n");
        return PTR_ERR(bmx160_thread);
    }

    return 0; // Success
}

static void bmx160_remove(struct i2c_client *client)
{
    dev_info(&client->dev, "Removing BMX160 driver\n");

    thread_run = false;
    if (bmx160_thread)
        kthread_stop(bmx160_thread);
}

static int __init bmx160_module_init(void)
{
    printk(KERN_INFO "bmx160: Initializing I2C driver\n");
    return i2c_add_driver(&bmx160_driver);
}

static void __exit bmx160_module_exit(void)
{
    i2c_del_driver(&bmx160_driver);
    printk(KERN_INFO "bmx160: Exiting I2C driver\n");
}

static int bmx160_read_accel_data(struct i2c_client *client)
{
    int ret;
    u8 buffer[6]; // X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB

    ret = i2c_smbus_read_i2c_block_data(client, BMX160_ACCEL_DATA_ADDR, 6, buffer);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to read accel data: %d\n", ret);
        return ret;
    }

    s16 ax = (buffer[1] << 8) | buffer[0];
    s16 ay = (buffer[3] << 8) | buffer[2];
    s16 az = (buffer[5] << 8) | buffer[4];

    dev_info(&client->dev, "Accel Raw Data => X: %d, Y: %d, Z: %d\n", ax, ay, az);

    return 0;
}

static int bmx160_read_gyro_data(struct i2c_client *client)
{
    int ret;
    u8 buffer[6]; // X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB

    ret = i2c_smbus_read_i2c_block_data(client, BMX160_GYRO_DATA_ADDR, 6, buffer);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to read gyro data: %d\n", ret);
        return ret;
    }

    s16 ax = (buffer[1] << 8) | buffer[0];
    s16 ay = (buffer[3] << 8) | buffer[2];
    s16 az = (buffer[5] << 8) | buffer[4];

    dev_info(&client->dev, "Gyro Raw Data => X: %d, Y: %d, Z: %d\n", ax, ay, az);

    return 0;
}

static int bmx160_read_megneto_data(struct i2c_client *client)
{
    int ret;
    u8 buffer[6]; // X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB

    ret = i2c_smbus_read_i2c_block_data(client, BMX160_MEGNETO_DATA_ADDR, 6, buffer);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to read megneto data: %d\n", ret);
        return ret;
    }

    s16 ax = (buffer[1] << 8) | buffer[0];
    s16 ay = (buffer[3] << 8) | buffer[2];
    s16 az = (buffer[5] << 8) | buffer[4];

    dev_info(&client->dev, "Megneto Raw Data => X: %d, Y: %d, Z: %d\n", ax, ay, az);

    return 0;
}

static int bmx160_polling_thread(void *data)
{
    struct i2c_client *client = (struct i2c_client *)data;

    while (!kthread_should_stop() && thread_run) {
        bmx160_read_accel_data(client);
        bmx160_read_gyro_data(client);
        bmx160_read_megneto_data(client);

        msleep(1000);  // Delay for 1 second
    }

    return 0;
}

/**********************************************************************/

static struct i2c_driver bmx160_driver = {
    .driver = {
        .name = "bmx160",
        .owner = THIS_MODULE,
    },
    .probe = bmx160_probe,
    .remove = bmx160_remove,
    .id_table = bmx160_id,
};

module_init(bmx160_module_init);
module_exit(bmx160_module_exit);

/**********************************************************************/
MODULE_LICENSE("GPL");
MODULE_AUTHOR("ronveliaBhavikGit");
MODULE_DESCRIPTION("BMX160 I2C linux device driver");
/**********************************************************************/