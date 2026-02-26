import sys
import argparse
import os
from tempfile import TemporaryDirectory
import xml.etree.ElementTree as et
from xml.etree.ElementTree import Element
import shutil
import subprocess
from io import BytesIO

class GowinProject:
    def __init__(self, project_path):
        self.prj_path = project_path
        self.prj_dir = os.path.dirname(self.prj_path)
        self.prj_user_path = project_path + '.user'
        self.result_files = {}

        tree = et.parse(self.prj_user_path)
        root: Element = tree.getroot()
        
        for i in root.findall('ResultFileList/*'):
            path = os.path.join(self.prj_dir, i.attrib['ResultFilePath'])
            self.result_files[i.attrib['ResultFileType']] = os.path.abspath(path)
        
    def get_bitstream_path(self):
        return self.result_files['RES.pnr.bitstream']
    
    def get_posp_path(self):
        db_path = self.result_files['RES.pnr.posp.bin']
        return os.path.splitext(db_path)[0] + '.posp'

def make_itcm(filepath, output):
    with open(filepath, 'rb') as f:
        data = f.read()

    itcm_file = [ open(os.path.join(output, f'itcm{i}'), 'w+') for i in range(4) ]
    for i in range(0, len(data), 4):
        dstr = "%08x" % int.from_bytes(data[i:i+4], 'little')
        itcm_file[0].write(dstr[6:8] + '\n')
        itcm_file[1].write(dstr[4:6] + '\n')
        itcm_file[2].write(dstr[2:4] + '\n')
        itcm_file[3].write(dstr[0:2] + '\n')

def bitstream2bin(filepath, output):
    buffer = BytesIO()
    b_temp = ''
    with open(filepath) as f:
        while line := f.readline():
            comments = line.find('//')
            if comments >= 0:
                line = line[:comments]
            line = line.strip()
            if not line:
                continue

            for c in line:
                b_temp += c
                if len(b_temp) == 8:
                    buffer.write(int(b_temp, 2).to_bytes())
                    b_temp = ''
    
    with open(output, 'wb+') as f:
        f.write(buffer.getvalue())


def main():
    temp_dir = TemporaryDirectory()
    
    parser = argparse.ArgumentParser(
        prog=sys.argv[0],
        description="比特流打包工具",
    )
    parser.add_argument('-p', '--project')
    parser.add_argument('-c', '--cbin')
    parser.add_argument('-o', '--output')

    args = parser.parse_args()

    c_bin_path = args.cbin

    project = GowinProject(os.path.abspath(args.project))
    shutil.copy(c_bin_path, temp_dir.name)
    shutil.copy(project.get_bitstream_path(), temp_dir.name)
    shutil.copy(project.get_posp_path(), temp_dir.name)

    # make_itcm("build/Application/HFLinkAPP.bin", temp_dir.name)

    merge_bit_exe = os.path.join(os.path.dirname(os.path.abspath(__file__)), "merge_bit.exe")

    with subprocess.Popen(
        [
            merge_bit_exe, 
            os.path.basename(c_bin_path),
            os.path.basename(project.get_bitstream_path()),
            "32",
            os.path.basename(project.get_posp_path())
        ],
        stdout=subprocess.DEVNULL,
        shell=True,
        cwd=temp_dir.name
        ) as pipe:
        exit_code = pipe.wait()
        ...

    if exit_code != 0:
        exit(exit_code)
    
    result_filename = 'new_' + os.path.basename(project.get_bitstream_path())
    result_path = os.path.join(temp_dir.name, result_filename)
    output_path = result_filename
    if args.output is not None:
        output_path = args.output
    fs_output_path = os.path.splitext(output_path)[0] + '.fs'
    
    if os.path.exists(fs_output_path):
        os.chmod(fs_output_path, 0o666)
    shutil.copy(result_path, fs_output_path)
    bitstream2bin(result_path, output_path)

        
if __name__ == "__main__":
    main()
