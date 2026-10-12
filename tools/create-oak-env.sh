#!/usr/bin/env bash

ROOT_DIR="$PIXI_PROJECT_ROOT/.packages"
CACHE_DIR="$ROOT_DIR/var/cache/apt/archives"
DEB="$CACHE_DIR/oak-viewer_1.6.3_amd64.deb"

mkdir -p "$CACHE_DIR"

wget -nc -q --show-progress -O "$DEB" \
  https://oak-viewer-releases.luxonis.com/data/1.6.3/debian_x86_64/viewer.deb

apt install --print-uris "$DEB" \
  | grep "^'" \
  | while read -r line; do

    url=$(echo "$line" | cut -d"'" -f2)
    rest=$(echo "$line" | sed "s|'[^']*' ||")

    filename=$(echo "$rest" | awk '{print $1}')
    size=$(echo "$rest" | awk '{print $2}')
    md5=$(echo "$rest" | sed -n 's/.*MD5Sum:\([a-fA-F0-9]\{32\}\).*/\1/p')

    if [[ -z "${filename:-}" ]]; then
        echo "Skipping malformed line: $line"
        continue
    fi

    filepath="$CACHE_DIR/$filename"
    echo "$filename"

    verify() {
        if [[ -n "$md5" ]]; then
            echo "${md5}  ${filepath}" | md5sum -c --status
        else
            return 0
        fi
    }

    if [[ -f "$filepath" ]]; then
        if verify; then
            continue
        else
            rm -f "$filepath"
        fi
    fi

    wget -nc -q --show-progress -O "$filepath" "$url"

    if verify; then
        echo "Done"
    else
        rm -f "$filepath"
        exit 1
    fi
done

for deb in "$CACHE_DIR"/*.deb; do
    echo "$deb"
    dpkg-deb -x "$deb" $ROOT_DIR
done
