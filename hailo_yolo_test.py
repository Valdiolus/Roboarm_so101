import cv2
import numpy as np

from hailo_platform import (
    HEF,
    VDevice,
    FormatType,
    HailoSchedulingAlgorithm,
)


HEF_PATH = "yolov11n.hef"
CONF_THRESHOLD = 0.35


def letterbox_rgb(image_rgb, dst_w, dst_h):
    """
    Resize while preserving aspect ratio.
    Returns:
        padded image
        scale
        x padding
        y padding
    """
    src_h, src_w = image_rgb.shape[:2]

    scale = min(dst_w / src_w, dst_h / src_h)

    new_w = int(src_w * scale)
    new_h = int(src_h * scale)

    resized = cv2.resize(
        image_rgb,
        (new_w, new_h),
        interpolation=cv2.INTER_LINEAR,
    )

    output = np.full(
        (dst_h, dst_w, 3),
        114,
        dtype=np.uint8,
    )

    pad_x = (dst_w - new_w) // 2
    pad_y = (dst_h - new_h) // 2

    output[
        pad_y:pad_y + new_h,
        pad_x:pad_x + new_w
    ] = resized

    return output, scale, pad_x, pad_y


class HailoYOLO:
    def __init__(self, hef_path):
        # Hailo device configuration
        params = VDevice.create_params()
        params.scheduling_algorithm = HailoSchedulingAlgorithm.ROUND_ROBIN
        params.group_id = "SHARED"

        self.device = VDevice(params)

        # Load HEF
        self.hef = HEF(hef_path)

        input_info = self.hef.get_input_vstream_infos()[0]

        self.input_h = input_info.shape[0]
        self.input_w = input_info.shape[1]

        print(
            f"Model input: "
            f"{self.input_w}x{self.input_h}"
        )

        # Create inference model
        self.model = self.device.create_infer_model(hef_path)
        self.model.set_batch_size(1)

        # We will send uint8 RGB frames
        self.model.input().set_format_type(FormatType.UINT8)

        # Get outputs as float32
        self.output_names = []

        for output_info in self.hef.get_output_vstream_infos():
            name = output_info.name
            self.output_names.append(name)

            self.model.output(name).set_format_type(
                FormatType.FLOAT32
            )

            print(
                "Output:",
                name,
                output_info.shape,
            )

        # Configure network once.
        # Do NOT configure on every frame.
        self.config_context = self.model.configure()
        self.configured_model = (
            self.config_context.__enter__()
        )

    def infer(self, image_rgb):
        """
        image_rgb: uint8 HxWx3 RGB
        """

        processed, scale, pad_x, pad_y = letterbox_rgb(
            image_rgb,
            self.input_w,
            self.input_h,
        )

        output_buffers = {}

        for name in self.output_names:
            shape = self.model.output(name).shape

            output_buffers[name] = np.empty(
                shape,
                dtype=np.float32,
            )

        bindings = self.configured_model.create_bindings(
            output_buffers=output_buffers
        )

        bindings.input().set_buffer(processed)

        error = []

        def callback(completion_info):
            if completion_info.exception is not None:
                error.append(completion_info.exception)

        self.configured_model.wait_for_async_ready(
            timeout_ms=10000
        )

        job = self.configured_model.run_async(
            [bindings],
            callback,
        )

        job.wait(10000)

        if error:
            raise RuntimeError(error[0])

        if len(self.output_names) == 1:
            result = bindings.output().get_buffer()
        else:
            result = {
                name: bindings.output(name).get_buffer()
                for name in self.output_names
            }

        return result, scale, pad_x, pad_y

    def close(self):
        if self.config_context is not None:
            self.config_context.__exit__(
                None,
                None,
                None,
            )


def decode_hailo_nms(
    output,
    image_shape,
    model_w,
    model_h,
    scale,
    pad_x,
    pad_y,
):
    """
    Hailo YOLO NMS output is organized by class.

    Each detection is approximately:

        [ymin, xmin, ymax, xmax, confidence]

    Coordinates are normalized to the model image.
    """

    image_h, image_w = image_shape[:2]

    detections = []

    for class_id, class_detections in enumerate(output):

        for det in class_detections:

            if len(det) < 5:
                continue

            ymin, xmin, ymax, xmax, score = det[:5]

            if score < CONF_THRESHOLD:
                continue

            # Normalized -> model coordinates
            x1 = xmin * model_w
            y1 = ymin * model_h
            x2 = xmax * model_w
            y2 = ymax * model_h

            # Remove letterbox padding
            x1 = (x1 - pad_x) / scale
            x2 = (x2 - pad_x) / scale
            y1 = (y1 - pad_y) / scale
            y2 = (y2 - pad_y) / scale

            # Clip to original image
            x1 = int(np.clip(x1, 0, image_w - 1))
            x2 = int(np.clip(x2, 0, image_w - 1))
            y1 = int(np.clip(y1, 0, image_h - 1))
            y2 = int(np.clip(y2, 0, image_h - 1))

            detections.append(
                {
                    "class_id": class_id,
                    "score": float(score),
                    "bbox": (x1, y1, x2, y2),
                }
            )

    return detections


def main():
    detector = HailoYOLO(HEF_PATH)

    #import image with opencv
    photo = cv2.imread("cam.jpg")
    #cap = cv2.VideoCapture(0)

    #if not cap.isOpened():
    #    raise RuntimeError("Could not open camera")

    try:
        while True:
            frame_bgr = photo

            # Hailo model expects RGB
            frame_rgb = cv2.cvtColor(
                frame_bgr,
                cv2.COLOR_BGR2RGB,
            )

            output, scale, pad_x, pad_y = detector.infer(
                frame_rgb
            )

            detections = decode_hailo_nms(
                output,
                frame_rgb.shape,
                detector.input_w,
                detector.input_h,
                scale,
                pad_x,
                pad_y,
            )

            for det in detections:
                x1, y1, x2, y2 = det["bbox"]
                class_id = det["class_id"]
                score = det["score"]

                cv2.rectangle(
                    frame_bgr,
                    (x1, y1),
                    (x2, y2),
                    (0, 255, 0),
                    2,
                )

                cv2.putText(
                    frame_bgr,
                    f"class={class_id} {score:.2f}",
                    (x1, max(y1 - 8, 20)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1,
                    cv2.LINE_AA,
                )
            #just save the image to disk instead of displaying it
            cv2.imwrite("output.jpg", frame_bgr)
            #cv2.imshow("Hailo YOLO11", frame_bgr)

            #if cv2.waitKey(1) & 0xFF == ord("q"):
            #    break

    finally:
        #cap.release()
        #cv2.destroyAllWindows()
        detector.close()


if __name__ == "__main__":
    main()