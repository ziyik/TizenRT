/****************************************************************************
 * kernel/sched/sched_getaffinity.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>

#include <sys/types.h>
#include <sched.h>
#include <assert.h>
#include <errno.h>

#include <tinyara/sched.h>

#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sched_getaffinity
 *
 * Description:
 *   sched_getaffinity() writes the affinity mask of the thread whose ID
 *   is pid into the cpu_set_t pointed to by mask.  The  cpusetsize
 *   argument specifies the size (in bytes) of mask.  If pid is zero, then
 *   the mask of the calling thread is returned.
 *
 *   This function is a simply wrapper around nxsched_get_affinity() that
 *   sets the errno value in the event of an error.
 *
 * Input Parameters:
 *   pid        - The ID of thread whose affinity set will be retrieved.
 *   cpusetsize - Size of mask.  MUST be sizeofcpu_set_t().
 *   mask       - The location to return the thread's new affinity set.
 *
 * Returned Value:
 *   0 if successful.  Otherwise, ERROR (-1) is returned, and errno is
 *   set appropriately:
 *
 *      ESRCH  The task whose ID is pid could not be found.
 *
 ****************************************************************************/

int sched_getaffinity(pid_t pid, size_t cpusetsize, FAR cpu_set_t *mask)
{
#ifdef CONFIG_SMP
  FAR struct tcb_s *tcb;
  int ret;

  DEBUGASSERT(cpusetsize == sizeof(cpu_set_t) && mask != NULL);

  /* Verify that the PID corresponds to a real task */

  sched_lock();
  if (pid == 0)
    {
      tcb = this_task();
    }
  else
    {
      tcb = sched_gettcb(pid);
    }

  if (tcb == NULL)
    {
      ret = -ESRCH;
    }
  else
    {
      /* Return the affinity mask from the TCB. */

      *mask = tcb->affinity;
      ret = OK;
    }

  sched_unlock();
  return ret;
#else
  /* In the absence of SMP, all tasks will always run on core 0 */
  return 0;
#endif
}
