#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$(realpath "$0")")";

# automagically refresh generated documents when doc source changes
while true; do
	inotifywait -r -e modify --exclude=".swp" . && make;
    echo -e "\e[42m################################\e[0m";
    echo -e "\e[42m# document generation finished #\e[0m";
    echo -e "\e[42m################################\e[0m";
done

