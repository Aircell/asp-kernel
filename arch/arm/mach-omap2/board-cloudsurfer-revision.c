/*
 * Aircell - Parse and report board revision kernel option
 * jkelly - March 2013
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>

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


void cloudsurfer_revision_setup(void) {

	entry = create_proc_entry("board_info", 0644, NULL);
	entry->read_proc = revision_read;
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
