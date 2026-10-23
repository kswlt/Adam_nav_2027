#!/usr/bin/env python3
"""Install built panel; optional local secret stays outside shared source/layouts."""
import argparse
import json
from pathlib import Path
import shutil
import zipfile


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--token-file', type=Path)
    parser.add_argument('--export-directory', type=Path)
    args = parser.parse_args()
    source = Path(__file__).parent/'foxglove'
    manifest = json.loads((source/'package.json').read_text(encoding='utf-8'))
    dist = source/'dist/extension.js'
    if not dist.exists():
        raise RuntimeError('先在foxglove目录 npm ci && npm run build')
    destination = Path.home()/'.foxglove-studio/extensions'/f"{manifest['publisher']}.{manifest['name']}-{manifest['version']}"
    (destination/'dist').mkdir(parents=True, exist_ok=True)
    shutil.copyfile(source/'package.json', destination/'package.json')
    code = dist.read_text(encoding='utf-8')
    if args.token_file:
        token = args.token_file.read_text(encoding='utf-8').strip()
        if not token or any(c not in 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_-' for c in token):
            raise ValueError('令牌格式无效')
        marker = 'frame.src = "http://127.0.0.1:8878/"'
        if marker not in code:
            raise ValueError('构建输出不匹配，未注入令牌')
        code = code.replace(marker, 'frame.src = '+json.dumps('http://127.0.0.1:8878/#'+token))
    (destination/'dist/extension.js').write_text(code, encoding='utf-8')
    if args.export_directory:
        args.export_directory.mkdir(parents=True, exist_ok=True)
        with zipfile.ZipFile(args.export_directory/'Adam_Z_operator-0.1.0.foxe', 'w', zipfile.ZIP_DEFLATED) as package:
            # Distributable never includes the private injected build.
            package.write(source/'package.json', 'package.json')
            package.write(dist, 'dist/extension.js')
        repo = Path(__file__).resolve().parents[2]
        shutil.copyfile(repo/'src/rm_nav_bringup/config/foxglove/z_operator.json', args.export_directory/'Adam_Z_operator.json')
        if args.token_file:
            url = 'http://127.0.0.1:8878/#'+token
            (args.export_directory/'Adam_Z_console.url').write_text('[InternetShortcut]\nURL='+url+'\n', encoding='utf-8')
    print('Local panel installed. Reload Foxglove; no private token in distributable package.')


if __name__ == '__main__':
    main()
