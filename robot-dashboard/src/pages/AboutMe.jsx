import React from "react";
import ReactMarkdown from "react-markdown";
import rehypeRaw from "rehype-raw";
import { Prism as SyntaxHighlighter } from "react-syntax-highlighter";
import { oneDark } from "react-syntax-highlighter/dist/esm/styles/prism";

import readme from "../../../README.md?raw";
import "./AboutMe.css";

export default function AboutMe() {
  return (
    <div className="markdown">
      <ReactMarkdown
        rehypePlugins={[rehypeRaw]}
        components={{
          img({ src, alt, ...props }) {
            return (
              <img
                src={`/${src.split("/").pop()}`}
                alt={alt}
                {...props}
              />
            );
          },

          a({ href, children, ...props }) {
            if (href?.endsWith(".mp4")) {
              return (
                <video
                  controls
                  width="600"
                  style={{ display: "block", margin: "20px auto" }}
                >
                  <source
                    src={`/${href.split("/").pop()}`}
                    type="video/mp4"
                  />
                  Your browser does not support video playback.
                </video>
              );
            }

            return (
              <a href={href} {...props}>
                {children}
              </a>
            );
          },

          code({ children, className }) {
            const language = className?.replace("language-", "");

            return (
              <SyntaxHighlighter
                language={language}
                style={oneDark}
              >
                {children}
              </SyntaxHighlighter>
            );
          },
        }}
      >
        {readme}
      </ReactMarkdown>
    </div>
  );
}