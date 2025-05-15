# chatbot-rnn
A toy chatbot powered by deep learning and trained on data from Reddit. Built on TensorFlow v2.13.0 and Python v3.8.17.

Here is a sample chat transcript (not cherry-picked). Lines that start with ">" are me; the lines without ">" are the chatbot's responses.

```
$ python chatbot.py
Creating model...
Restoring weights...

> Hi, how are you?
 I'm not the only one here that's having fun with you, it's all me.
> Who else is with you?
 Haha. I'm not the one who made the comment. I'm the one who said that and didn't think that much of anything. 
> Oh, I see. So what's new?
 Nothing. I just thought it was a bit of an overreaction. It's been a while since I've read it. 
> It's all good
 It's all good. It's one of those things that makes me smile :)
> That's great. This is actually my demo chat, so it's nice that you're keeping things positive!
 Thank you :) 
> You're welcome! It's a nice change from the way that you sometimes are...
 *blushes* Sorry, I should have clarified that I wasn't clear. I apologize for my ignorance.
> Sooo, where are you from?             
 I am from South America. Where you from?
> California. Do you like it in South America?
 I live in Australia, but south of Florida is one of the reasons I live in Florida. I've never been there, but I've never been there. 
> I see, a true citizen of the world
 Nope, I've never heard of it. I just spent a lot of time there. 
> Spend a lot of time where?
 I've never been there before, but I've heard it's about an hour and a half away. Maybe there's some sort of party called "Southern" but that's about it. 
> Maybe there is, that's a great observation.
 Not to mention it's all about being from another country.
> All right, guess I should wrap this up, nice talking with you
 I'm not going to argue with you. You're doing it wrong.
> Some might call that arguing. Any final words?
 Don't do that. I don't need to. 
 ```

## Get Started

- **Install [TensorFlow](http://www.tensorflow.org/)** for Python 3. To run a pretrained model, the CPU-only installation should suffice. If you want to train your own models, you'll need the GPU installation of TensorFlow (and a powerful CUDA-compatible GPU).

- Clone this project to your computer.

### Run my pre-trained model

- **Download [my pre-trained model](https://drive.google.com/uc?id=1rRRY-y1KdVk4UB5qhu7BjQHtfadIOmMk&export=download)** (2.3 GB). The zip file extracts into a folder named "reddit". Place that folder into the "models" directory of this project.

- **Run the chatbot**. Open a terminal session and run `python chatbot.py`. Warning: this pre-trained model was trained on a diverse set of frequently off-color Reddit comments. It can (and eventually will) say things that are offensive, disturbing, bizarre or sexually explicit. It may insult minorities, it may call you names, it may accuse you of being a pedophile, it may try to seduce you. Please don't use the chatbot if these possibilities would distress you!

Try playing around with the arguments to `chatbot.py` to obtain better samples:

- **beam_width**: By default, `chatbot.py` will use beam search with a beam width of 2 to sample responses. Set this higher for more careful, more conservative (and slower) responses, or set it to 1 to disable beam search.

- **temperature**: At each step, the model ascribes a certain probability to each character. Temperature can adjust the probability distribution. 1.0 is neutral (and the default), lower values increase high probability values and decrease lower probability values to make the choices more conservative, and higher values will do the reverse. Values outside of the range of 0.5-1.5 are unlikely to give coherent results.

- **top-n**: At each step, zero out the probability of all possible characters except the *n* most likely. Disabled by default.

- **relevance**: Two models are run in parallel: the primary model and the mask model. The mask model is scaled by the relevance value, and then the probabilities of the primary model are combined according to equation 9 in [Li, Jiwei, et al. "A diversity-promoting objective function for neural conversation models." arXiv preprint arXiv:1510.03055 (2015)](https://arxiv.org/abs/1510.03055). The state of the mask model is reset upon each newline character. The net effect is that the model is encouraged to choose a line of dialogue that is most relevant to the prior line of dialogue, even if a more generic response (e.g. "I don't know anything about that") may be more absolutely probable. Higher relevance values put more pressure on the model to produce relevant responses, at the cost of the coherence of the responses. Going much above 0.4 compromises the quality of the responses. Setting it to a negative value disables relevance, and this is the default, because I'm not confident that it qualitatively improves the outputs and it halves the speed of sampling.

These values can also be manipulated during a chat, and the model state can be reset, without restarting the chatbot:

```
$ python chatbot.py
Creating model...
Restoring weights...

> --temperature 1.3
[Temperature set to 1.3]

> --relevance 0.3
[Relevance set to 0.3]

> --relevance -1
[Relevance disabled]

> --topn 2
[Top-n filtering set to 2]

> --topn -1
[Top-n filtering disabled]

> --beam_width 5
[Beam width set to 5]

> --reset
[Model state reset]
```

