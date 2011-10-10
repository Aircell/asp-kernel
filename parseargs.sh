#!/bin/bash -eu

clean=""
quiet=""

parseargs() {
  args=${@:-}


  if ! tty -s; then
    args+=" -q"
  fi

  if grep -q q <<< $args; then
    grep -q s <<< $args &&
      die "'$0 $args': can't be both quiet and status-only"
  fi

  while getopts ":qcs" opt $args; do
    case $opt in
      s)		# status only
        status=$opt
        exec  >$0.OUT
        exec 2>$0.ERR
        ;;
      q)                # completely silent
	quiet=$opt
        exec  >$0.OUT
        exec 2>$0.ERR
        ;;
      c)
        clean=$opt
        ;;
      *)
        die $usage
        ;;
    esac

    if [ "$quiet" ]; then
      exec 3>$0.STATUS
    else
      exec 3>/dev/tty
    fi
  done
}

parseargs $@
