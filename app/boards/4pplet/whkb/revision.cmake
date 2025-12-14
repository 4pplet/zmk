# SPDX-License-Identifier: MIT

set(BOARD_REVISIONS "rev_b" "rev_b_jp")
if(NOT DEFINED BOARD_REVISION)
  set(BOARD_REVISION "rev_b")
else()
  if(NOT BOARD_REVISION IN_LIST BOARD_REVISIONS)
    message(FATAL_ERROR "${BOARD_REVISION} is not a valid revision for whkb. Accepted revisions: ${BOARD_REVISIONS}")
  endif()
endif()
