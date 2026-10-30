#!/usr/bin/env python3
"""Make annotated, concatenated RGB review videos from our LeRobot v3 export.

No ROS, robot access, source modification or LeRobot Python dependency needed.
Requires ffmpeg/ffprobe with libass. Output must be a new directory.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import csv
from fractions import Fraction
import hashlib
import json
from pathlib import Path
import subprocess


def probe(path, count=False):
    cmd=['ffprobe','-v','error','-select_streams','v:0']
    if count:cmd+=['-count_frames']
    cmd+=['-show_entries','stream=width,height,nb_frames,nb_read_frames,r_frame_rate,start_time,duration','-of','json',str(path)]
    return json.loads(subprocess.check_output(cmd,text=True))['streams'][0]


def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda:f.read(8*1024*1024),b''):h.update(b)
    return h.hexdigest()


def ass_time(frame,fps):
    # Change labels between adjacent frames; centisecond rounding cannot move
    # a caption into the following video frame at the 30 Hz export rate.
    ticks=round(max(0,frame-.5)*100/fps)
    seconds,centis=divmod(ticks,100);minutes,seconds=divmod(seconds,60);hours,minutes=divmod(minutes,60)
    return f'{hours}:{minutes:02d}:{seconds:02d}.{centis:02d}'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('dataset',type=Path)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args();root=args.dataset.resolve()
    output=(args.output or root.with_name(root.name+'_review')).resolve()
    if output==root or output.is_relative_to(root):parser.error('Output must be outside the dataset')
    if output.exists():parser.error('Output directory already exists; nothing will be overwritten')
    info_path=root/'meta/info.json';segments_path=root/'provenance/segments.json'
    info=json.loads(info_path.read_text());segments=json.loads(segments_path.read_text())
    fps=int(info['fps']);total=sum(s['frames'] for s in segments)
    if not segments or [s['dataset_episode_index'] for s in segments]!=list(range(len(segments))):
        parser.error('Expected chronologically ordered contiguous segment indices from our exporter')
    if total!=info['total_frames']:parser.error('Segment frame counts do not match metadata')
    sources={role:sorted((root/'videos'/('observation.images.'+role)).rglob('*.mp4')) for role in ('scene','wrist')}
    shapes={};hashes={str(p):digest(p) for p in (info_path,segments_path)}
    for role,paths in sources.items():
        if not paths:parser.error(f'Missing {role} RGB video')
        count=0
        for path in paths:
            stream=probe(path);count+=int(stream['nb_frames'])
            if Fraction(stream['r_frame_rate'])!=fps:parser.error('Expected constant dataset FPS')
            shape=(int(stream['width']),int(stream['height']))
            if role in shapes and shapes[role]!=shape:parser.error('Changing video resolution')
            shapes[role]=shape;hashes[str(path)]=digest(path)
        if count!=total:parser.error(f'{role} video frame count mismatch')
    output.mkdir(parents=True)
    chapters=[';FFMETADATA1'];cursor=0;index=[]
    for s in segments:
        end=cursor+s['frames']
        title=f"Source episode {s['source_episode_number']} | LeRobot segment {s['dataset_episode_index']}"
        chapters+=['[CHAPTER]',f'TIMEBASE=1/{fps}',f'START={cursor}',f'END={end}',f'title={title}']
        index.append(dict(s,start_frame=cursor,end_frame_exclusive=end,start_seconds=cursor/fps,end_seconds=end/fps))
        cursor=end
    (output/'chapters.ffmetadata').write_text('\n'.join(chapters)+'\n')
    with (output/'episode_index.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(index[0]));writer.writeheader();writer.writerows(index)

    def export(role):
        width,height=shapes[role]
        caption=['[Script Info]','ScriptType: v4.00+',f'PlayResX: {width}',f'PlayResY: {height}',
            '[V4+ Styles]',
            'Format: Name, Fontname, Fontsize, PrimaryColour, SecondaryColour, OutlineColour, BackColour, Bold, Italic, Underline, StrikeOut, ScaleX, ScaleY, Spacing, Angle, BorderStyle, Outline, Shadow, Alignment, MarginL, MarginR, MarginV, Encoding',
            'Style: Default,DejaVu Sans,16,&H00FFFFFF,&H00FFFFFF,&H80000000,&H80000000,0,0,0,0,100,100,0,0,3,2,0,7,8,8,8,1',
            '[Events]','Format: Layer, Start, End, Style, Name, MarginL, MarginR, MarginV, Effect, Text']
        for row in index:
            label=f"{role.upper()} | Source episode {row['source_episode_number']} | LeRobot segment {row['dataset_episode_index']}"
            caption.append(f"Dialogue: 0,{ass_time(row['start_frame'],fps)},{ass_time(row['end_frame_exclusive'],fps)},Default,,0,0,0,,{label}")
        subtitle=role+'_labels.ass';(output/subtitle).write_text('\n'.join(caption)+'\n')
        paths=sources[role];inputs=['-i',str(paths[0])]
        if len(paths)>1:
            listing=output/(role+'_inputs.ffconcat')
            listing.write_text('ffconcat version 1.0\n'+''.join("file '"+str(p).replace("'", "'\\''")+"'\n" for p in paths))
            inputs=['-f','concat','-safe','0','-i',str(listing)]
        filename=role+'_all_episodes.mp4'
        cmd=['ffmpeg','-nostdin','-hide_banner','-loglevel','error','-n',*inputs,
            '-f','ffmetadata','-i','chapters.ffmetadata','-map','0:v:0','-map_metadata','1','-map_chapters','1',
            '-vf','subtitles='+subtitle,'-c:v','libx264','-preset','fast','-crf','18','-pix_fmt','yuv420p',
            '-threads','2','-an','-movflags','+faststart','-metadata','title='+role.title()+' - trimmed cable demonstrations',
            '-progress',role+'.progress',filename]
        with (output/(role+'.log')).open('w') as log:
            subprocess.run(cmd,cwd=output,stdout=log,stderr=subprocess.STDOUT,check=True)
        # Decode every frame, rather than trusting only the MP4 frame-count tag.
        result=probe(output/filename,count=True)
        if int(result['nb_read_frames'])!=total or int(result['nb_frames'])!=total:
            raise RuntimeError(f'{role} output failed full-frame decode count: {result}')
        if abs(float(result['duration'])-total/fps)>.001:raise RuntimeError('Duration mismatch')
        print(json.dumps({'completed':role,'frames':total,'seconds':total/fps,'path':str(output/filename)}),flush=True)
        return dict(role=role,file=filename,bytes=(output/filename).stat().st_size,probe=result,command=cmd)

    with ThreadPoolExecutor(max_workers=2) as pool:results=list(pool.map(export,('scene','wrist')))
    for path,original in hashes.items():
        if digest(Path(path))!=original:raise RuntimeError('Dataset source changed: '+path)
    report=dict(dataset=str(root),fps=fps,frames=total,duration_seconds=total/fps,segments=len(segments),
        source_episode_numbers=sorted({s['source_episode_number'] for s in segments}),
        dataset_sources_unchanged=True,input_sha256=hashes,results=results,
        note='Review copies with burnt-in labels; RGB re-encoded with H.264 CRF 18. No removed pauses restored.')
    (output/'review_manifest.json').write_text(json.dumps(report,indent=2)+'\n')
    (output/'README.md').write_text('# Trimmed cable review videos\n\n'
        f'Both views cover {total} frames at {fps} Hz ({total/fps:.3f} seconds).\n'
        'Scene and wrist are separate complete videos in identical segment order.\n'
        'Labels show the original source episode and derived LeRobot segment index.\n'
        'Chapter markers and episode_index.csv provide the same mapping.\n'
        'The dataset and raw recordings are unchanged. These are annotated, lossy review copies.\n')
    print(json.dumps({'output':str(output),'validated':True}),flush=True)


if __name__=='__main__':main()
