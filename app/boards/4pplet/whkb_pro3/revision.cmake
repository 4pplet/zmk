# SPDX-License-Identifier: MIT

set(BOARD_REVISIONS "ansi" "jp")
if(NOT DEFINED BOARD_REVISION)
  set(BOARD_REVISION "ansi")
else()
  if(NOT BOARD_REVISION IN_LIST BOARD_REVISIONS)
    message(FATAL_ERROR "${BOARD_REVISION} is not a valid revision for whkb_pro3. Accepted revisions: ${BOARD_REVISIONS}")
  endif()
endif()
