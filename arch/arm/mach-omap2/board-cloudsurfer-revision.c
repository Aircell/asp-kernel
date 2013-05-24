/*
 * Aircell - Parse and report board revision kernel option
 * jkelly - March 2013
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <asm/mach-types.h>

static struct proc_dir_entry *entry;
static int board_revision;


static int revision_read(char *page, char **start,
                            off_t off, int count,
                            int *eof, void *pdata)
{
	int len;

        len = sprintf(page, "Board-Revision: %d\n", board_revision);

        return len;
}

/*
 * /sys/board_properties/revision
 */
static ssize_t cloudsurfer_revision_show(struct kobject *kobj,
			       struct kobj_attribute *attr, char *buf)
{
	const char *revstr;
	if (machine_is_cloudsurfer_p3() || machine_is_dm3730_som_lv()) {
		// machine_is_dm3730_som_lv is for old u-boot
		// P3 board goes in Rev A phone
		// Rev A board goes in Rev B phone (REALLY???!!!)
		revstr = "Rev A";
	} else if (machine_is_cloudsurfer_reva()) {
		revstr = "Rev B";
	} else {
		revstr = "unknown";
	}
	return sprintf(buf, "%s\n", revstr);
}

static struct kobj_attribute cloudsurfer_revision_attr = {
	.attr = {
		.name = "revision",
		.mode = S_IRUGO,
	},
	.show = &cloudsurfer_revision_show,
};

static struct attribute *cloudsurfer_properties_attrs[] = {
	&cloudsurfer_revision_attr.attr,
	NULL
};

static struct attribute_group cloudsurfer_properties_attr_group = {
	.attrs = cloudsurfer_properties_attrs,
};

void cloudsurfer_revision_setup(void) {
	struct kobject *properties_kobj;
	int ret;

	entry = create_proc_entry("board_info", 0644, NULL);
	entry->read_proc = revision_read;

	// Also create /sys/board_properties/revision
	ret = 0;
	properties_kobj = kobject_create_and_add("board_properties", NULL);
	if (properties_kobj)
		ret = sysfs_create_group(properties_kobj,
					 &cloudsurfer_properties_attr_group);
	if (!properties_kobj || ret)
		pr_err("failed to create board_properties\n");

        if (machine_is_cloudsurfer_p3() || machine_is_dm3730_som_lv()) {
		board_revision=0;
        } else if (machine_is_cloudsurfer_reva()) {
		board_revision=1;
        } else {
		board_revision=-1;
        }

}

static int __init setup_board_revision(char *str)
{
        get_option(&str, &board_revision);
        return 1;
}

int cloudsurfer_board_revision(void) {
	return board_revision;
}
EXPORT(cloudsurfer_board_revision);

__setup("board_revision=", setup_board_revision);
