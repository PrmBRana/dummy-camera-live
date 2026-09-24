/****************************************************************************
 * apps/examples/littlefs/read_main.c
 *
 * NSH shortcut command: 'read [hk1|hk2]'
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>

extern int littlefs_main(int argc, char *argv[]);

int read_main(int argc, char *argv[])
{
  char *lfs_argv[4];
  lfs_argv[0] = "littlefs";
  lfs_argv[1] = "read";
  lfs_argv[2] = (argc > 1) ? argv[1] : NULL;
  lfs_argv[3] = NULL;
  return littlefs_main((argc > 1) ? 3 : 2, lfs_argv);
}
