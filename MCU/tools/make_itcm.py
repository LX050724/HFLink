import sys
import argparse
import os

def main():
    parser = argparse.ArgumentParser(
        prog=sys.argv[0],
        description="bin转item",
    )
    parser.add_argument('filename')
    parser.add_argument('-o', '--output')
    args = parser.parse_args()

    make_itcm(args.filename, args.output)


def make_itcm(filepath, output):
    if not os.path.isdir(output):
        print('输出目录应该为文件夹')
        exit(1)

    with open(filepath, 'rb') as f:
        data = f.read()

    itcm_file = [ open(os.path.join(output, f'itcm{i}'), 'w+') for i in range(4) ]
    for i in range(0, len(data), 4):
        dstr = "%08x" % int.from_bytes(data[i:i+4], 'little')
        itcm_file[0].write(dstr[6:8] + '\n')
        itcm_file[1].write(dstr[4:6] + '\n')
        itcm_file[2].write(dstr[2:4] + '\n')
        itcm_file[3].write(dstr[0:2] + '\n')
        
if __name__ == "__main__":
    main()