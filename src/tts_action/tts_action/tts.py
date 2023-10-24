"""
models list: https://github.com/espnet/espnet_model_zoo/blob/master/espnet_model_zoo/table.csv
"""
import glob
import os
import time
import kaldiio
import numpy as np
from espnet2.bin.tts_inference import Text2Speech
from espnet2.utils.types import str_or_none
import torch
from espnet_model_zoo.downloader import ModelDownloader
import soundfile as sf


class TTS:
    def __init__(self,
                 tag='kan-bayashi/vctk_full_band_multi_spk_vits',
                 spembs=None,
                 sids=None,
                 speech=None,
                 vocoder_tag="none",
                 device=("cuda" if torch.cuda.is_available() else "cpu"),
                 ):
        self.text2speech = Text2Speech.from_pretrained(
            model_tag=str_or_none(tag),
            vocoder_tag=str_or_none(vocoder_tag),
            device="cuda" if torch.cuda.is_available() else "cpu",
            # Only for Tacotron 2 & Transformer
            threshold=0.5,
            # Only for Tacotron 2
            minlenratio=0.0,
            maxlenratio=10.0,
            use_att_constraint=False,
            backward_window=1,
            forward_window=3,
            # Only for FastSpeech & FastSpeech2 & VITS
            speed_control_alpha=1.0,
            # Only for VITS
            noise_scale=0.333,
            noise_scale_dur=0.333,
        )
        self.spembs = spembs
        self.sids = sids
        self.speech = speech
        self.ModelDownloader = ModelDownloader()
        self.model_dir = os.path.dirname(self.ModelDownloader.download_and_unpack(tag)["train_config"])
        self.model_dir = "/usr/local/lib/python3.8/dist-packages/espnet_model_zoo/98c32ce502238e314b60837f457fbb09/exp/tts_train_full_band_multi_spk_vits_raw_phn_tacotron_g2p_en_no_space"
        print(f"tts model_dir = {self.model_dir}")
        print("初始化tts模型")
        # self.text2speech = Text2Speech(
        #     # device=device,
        #     train_config="/usr/local/lib/python3.8/dist-packages/espnet_model_zoo/98c32ce502238e314b60837f457fbb09/exp/tts_train_full_band_multi_spk_vits_raw_phn_tacotron_g2p_en_no_space/config.yaml",
        #     model_file="/usr/local/lib/python3.8/dist-packages/espnet_model_zoo/98c32ce502238e314b60837f457fbb09/exp/tts_train_full_band_multi_spk_vits_raw_phn_tacotron_g2p_en_no_space/train.total_count.ave_10best.pth",
        # )
        # self.callback()
        print("tts模型初始化完成")

    def callback(self):
        if self.text2speech.use_spembs:
            xvector_ark = \
                [p for p in glob.glob(f"{self.model_dir}/../../dump/**/spk_xvector.ark", recursive=True) if "tr" in p][
                    0]
            xvectors = {k: v for k, v in kaldiio.load_ark(xvector_ark)}
            spks = list(xvectors.keys())

            # randomly select speaker
            random_spk_idx = np.random.randint(0, len(spks))
            spk = spks[random_spk_idx]
            self.spembs = xvectors[spk]
            print(f"selected spk: {spk}")
        if self.text2speech.use_sids:
            spk2sid = glob.glob(
                f"{self.model_dir}/../../dump/**/spk2sid", recursive=True)[0]
            with open(spk2sid) as f:
                lines = [line.strip() for line in f.readlines()]
            sid2spk = {int(line.split()[1]): line.split()[0] for line in lines}

            # randomly select speaker
            self.sids = np.array(np.random.randint(1, len(sid2spk)))
            spk = sid2spk[int(self.sids)]
            print(f"selected spk: {spk}")

        if self.text2speech.use_speech:
            # you can change here to load your own reference speech
            # e.g.
            # import soundfile as sf
            # speech, fs = sf.read("/path/to/reference.wav")
            # speech = torch.from_numpy(speech).float()
            self.speech = torch.randn(50000, ) * 0.01

    def generate_text_to_speech(self, text):
        torch.cuda.empty_cache()
        # return self.text2speech(text=text, speech=self.speech, sids=self.sids, spembs=self.spembs)["wav"].view(
        #     -1).cpu().numpy()
        return self.text2speech(text=text, sids=np.array(83))["wav"].numpy()


if __name__ == '__main__':
    with torch.no_grad():
        start = time.time()
        output_file_path = "output.wav"  # 定义输出文件路径
        # 把服务端给的结果写入wav文件
        speechdata = TTS().generate_text_to_speech(text="Hello World! Hello Python !")
        print(speechdata.dtype)
        print(TTS().text2speech.fs)
        sf.write(output_file_path, speechdata, TTS().text2speech.fs)
