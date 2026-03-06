#!/usr/bin/env bash

# This script must be sourced so exports land in the current shell.
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "Run this with: source ${BASH_SOURCE[0]}"
  exit 1
fi

export PX4_HOME_LAT="44.2253"
export PX4_HOME_LON="-76.4951"
export PX4_HOME_ALT="0.0"
