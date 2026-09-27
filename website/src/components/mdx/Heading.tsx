type HeadingProps = React.ComponentProps<"h2">;

/**
 * h2–h4 with a "#" link that appears on hover/focus, for deep-linking.
 * Ids come from rehype-slug (next.config.ts); headings without one render plain.
 */
function make(Tag: "h2" | "h3" | "h4") {
  function Heading({ id, children, ...rest }: HeadingProps) {
    return (
      <Tag id={id} {...rest}>
        {children}
        {id && (
          <a className="heading-anchor" href={`#${id}`} aria-label="Link to this section">
            #
          </a>
        )}
      </Tag>
    );
  }
  Heading.displayName = `Mdx${Tag.toUpperCase()}`;
  return Heading;
}

export const H2 = make("h2");
export const H3 = make("h3");
export const H4 = make("h4");
