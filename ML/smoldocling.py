#!/usr/bin/env python3
# ocr reader using smoldocling
#
import rich_click as click
import torch
from transformers import AutoModelForVision2Seq, AutoProcessor
from docling_core.types.doc import DoclingDocument
from docling_core.types.doc.document import DocTagsDocument
from transformers.image_utils import load_image

# class instance wrapper for the smodocling ocr
class SmolDoc():
    model_id = "ds4sd/SmolDocling-256M-preview"

    def __init__(self):
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        print(self.device)

    def load_model(self):
        self.processor = AutoProcessor.from_pretrained(self.model_id)
        self.model = AutoModelForVision2Seq.from_pretrained(
            self.model_id,
            torch_dtype=torch.bfloat16,
            _attn_implementation="flash_attention_2" if self.device == "cuda" else "eager",
        ).to(self.device)

    def vision(self, image_path, out_file):
        image = load_image(image_path)

        messages = [
            {"role": "user", "content": [
                {"type": "image"},
                {"type": "text", "text": "Convert this page to docling."}
            ]}
        ]

        prompt = self.processor.apply_chat_template(messages, add_generation_prompt=True)
        inputs = self.processor(text=prompt, images=image, return_tensors="pt").to(self.device)

        generated_ids = self.model.generate(**inputs, max_new_tokens=4096)
        prompt_length = inputs.input_ids.shape[1]
        trimmed_generated_ids = generated_ids[:, prompt_length:]

        doctags = self.processor.batch_decode(
            trimmed_generated_ids,
            skip_special_tokens=False,
        )[0]

        doctags_doc = DocTagsDocument.from_doctags_and_image_pairs([doctags], [image])
        doc = DoclingDocument(name="Document")
        doc.load_from_doctags(doctags_doc)
        print(doc.export_to_markdown())

        doc.save_as_markdown(out_file)

@click.command()
@click.option("--infile_name", default="./document.jpg", help="Picture to perform ocr on")
@click.option("--outfile_name", default="./output.md", help="Picture to perform ocr on")
def main(infile_name: str, outfile_name: str):
    SDS = SmolDoc()
    SDS.load_model()
    SDS.vision(infile_name, outfile_name)
    
if __name__ == "__main__":
    main()

